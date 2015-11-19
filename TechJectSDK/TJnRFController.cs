/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 */


// used to configure the nRF (initialization and Data writing & reading)

using System;
using System.Collections.Concurrent;
using System.Threading;
using System.Diagnostics;

namespace TechJectDF
{
    public class PacketReceivedEventArgs : EventArgs
    {
        TJPacket packet;

        public PacketReceivedEventArgs(TJPacket packet)
        {
            this.packet = packet;
        }

        public TJPacket Packet
        { get { return packet; } }
    }

    public class TJnRFController
    {
        public enum RFOpMode
        {
            RFRXMode,
            RFTXMode
        }

        public enum nRFChID
        {
            chID0 = 0, // 
            chID1 = 1, // 
            chID2 = 2, // 
            chID3 = 3, // 
            chID4 = 4, // 
            chID5 = 5, // 
        }

#if DEBUG
        private static Stopwatch profileSw = new Stopwatch();
#endif

        private static TJUSBToSPIInterface transceiver;
        private static TJGPIOPins IRQPin = TJGPIOPins.TJGPIO_0;    
        private static TJGPIOPins CEPin = TJGPIOPins.TJGPIO_1;  

        public delegate void PacketReceivedHandler(PacketReceivedEventArgs packet);
        public static event PacketReceivedHandler PacketReceived;

        private static Thread MonitoringThread = null;
        private static Thread EventDispatchingThread = null;
        private static int MissCount = 0;

        /// <summary>
        /// Holds payloads to be sent to the dragonfly.
        /// </summary>
        private static ConcurrentQueue<byte[]> TXPayloads;

        /// <summary>
        /// Holds packets that have not been processed yet
        /// </summary>
        private static ConcurrentQueue<byte[]> NewPackets;

        private static bool stopMonitoring;
        private static RFOpMode _Mode;

        public static RFOpMode Mode
        {
            get { return _Mode; }
            set { _Mode = value; }
        }

        /// <summary>
        /// Returns true if the nRF's IRQ is active
        /// </summary>
        public static bool IsIRQActive
        {
            get { return transceiver.GetInputGPIOPinValue(IRQPin) == 0; }  // IRQ pin is active low
        }

        /// <summary>
        /// Starts connection with nRF and initializes it in RX mode. If init is successful, it starts listening for data
        /// and returns true
        /// </summary>
        /// <param name="handler">If a valid delegate is specified, it will be called each time the nRF receives a packet.</param>
        /// <returns></returns>
        public static bool Init(int cont, byte pipe, PacketReceivedHandler handler = null)
        {
            // Comment/uncomment the following lines to select the USB-to-SPI interface to use. Make sure the pin definitions above match th
            // actual hardware connections.

            // Uncomment the following two lines to use with MCP2210/Aardvark... make sure the IRQ and CE cables math the GPIO definitions
            if (cont == 0)
            {
                transceiver = new TJMCP2210SPITransceiver();

                IRQPin = TJGPIOPins.TJGPIO_2;    // Uncomment for MCP2210
                CEPin = TJGPIOPins.TJGPIO_1;     // Uncomment for MCP2210
            }
            else if (cont == 1)
            {
                transceiver = new TJAardvarkSPITransceiver();

                IRQPin = TJGPIOPins.TJGPIO_0;      // Uncomment for aardvark
                CEPin = TJGPIOPins.TJGPIO_1;       // Uncomment for aardvark
            }

            if (transceiver.Init())
            {
                transceiver.SetGPIODirection(IRQPin);
                Mode = RFOpMode.RFRXMode;
                ConfigureRF(pipe);

                if (handler != null)
                    PacketReceived = handler;

				TXPayloads = new ConcurrentQueue<byte[]>();
				NewPackets = new ConcurrentQueue<byte[]>();

                StartDispatchThreads();
                
                return true;
            }

            return false;
        }

        /// <summary>
        /// Starts the thread used to monitor the nRF and retrieve packets from its buffer, as well as 
        /// a thread used to dispatch packets to all PacketReceivedHandler delegates.
        /// </summary>
        static void StartDispatchThreads()
        {
            if (MonitoringThread != null && MonitoringThread.IsAlive)
            {
                MonitoringThread.Abort();
                MonitoringThread.Join();
            }

            stopMonitoring = false;

            EventDispatchingThread = new Thread(new ThreadStart(ProcessPackets));
            EventDispatchingThread.Start(); ;
            MonitoringThread = new Thread(new ThreadStart(MonitorIRQ));
            MonitoringThread.Start();
        }

        /// <summary>
        /// Writes a single byte to the specified nRF register through SPI
        /// </summary>
        /// <param name="addr">Register address</param>
        /// <param name="value">Register value to write</param>
        /// <returns></returns>
        private static byte WriteRegisterByte(byte addr, byte value)
        {
            byte[] outBytes = { (byte)(0x20 | (addr & 0x1F)), value };
            byte[] inBytes = { 0, 0 };

            transceiver.ReadWriteSPI(ref outBytes, ref inBytes);

            return inBytes[0];
        }

        /// <summary>
        /// Writes multiple bytes to a specified nRF register through SPI
        /// </summary>
        /// <param name="addr">The register address</param>
        /// <param name="value">The byte array to write</param>
        private static void WriteRegisterMultiByte(byte addr, byte[] value)
        {
            SendMultiByteCommand((byte)(0x20 | (addr & 0x1F)), ref value);
        }

        /// <summary>
        /// Sends a single byte command through SPI (e.g. flush rx fifo command)
        /// </summary>
        /// <param name="command">The command code</param>
        private static void SendSingleByteCommand(byte command)
        {
            byte[] outByte = { command };
            transceiver.WriteSPI(ref outByte);
        }

        /// <summary>
        /// Sends a command along with multibyte data (e.g. send payload command).
        /// </summary>
        /// <param name="command">Command code</param>
        /// <param name="data">Data to send along with command</param>
        private static void SendMultiByteCommand(byte command, ref byte[] data)
        {
            byte[] outBytes = new byte[data.Length + 1];

            outBytes[0] = command;
            Buffer.BlockCopy(data, 0, outBytes, 1, data.Length);

            transceiver.WriteSPI(ref outBytes);
        }

        /// <summary>
        /// Reads a single-byte register through SPI
        /// </summary>
        /// <param name="addr">The register address</param>
        /// <returns>The value of the register</returns>
        private static byte ReadRegisterByte(byte addr)
        {
            byte[] outByte = { (byte)(0x1F & addr), 0 };
            byte[] inByte = { 0, 0 };

            transceiver.ReadWriteSPI(ref outByte, ref inByte);

            return inByte[1];
        }

        /// <summary>
        /// Reads the payload width register.
        /// </summary>
        /// <returns>The payload width (number of bytes of current RX payload.. should always be 32 in our case)</returns>
        private static byte ReadPayloadWidth()
        {
            byte[] outByte = { 0x60, 0 };
            byte[] inByte = { 0, 0 };

            transceiver.ReadWriteSPI(ref outByte, ref inByte);

            return inByte[1];
        }

        private static void ConfigureRF(byte id = (byte)nRFChID.chID1)
        {
            switch (_Mode)
            {
                case RFOpMode.RFRXMode:
                    ConfigureRFRX(id);
                    break;
                case RFOpMode.RFTXMode:
                    ConfigureRFTX();
                    break;
            }
        }

        /// <summary>
        /// Configures the nRF for TX mode... not implemented yet, as we don't need it. To send data, we are piggybacking
        /// packets on returned ACK packets.
        /// </summary>
        private static void ConfigureRFTX()
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Configures the nRF for RX mode 
        /// </summary>
        private static void ConfigureRFRX(byte id = (byte)nRFChID.chID1)
        {
            DeassertCE();

            WriteRegisterByte(0x00, 0x3D);      // RX data ready interrupt enabled, 1-byte CRC, power down, PRX
            WriteRegisterByte(0x01, 0x03);      // Auto-ack enabled all pipes
            WriteRegisterByte(0x02, 0x03);      // Data pipe x enabled
            WriteRegisterByte(0x03, 0x01);      // 3-byte RX/TX address field width
            WriteRegisterByte(0x04, 0x1F);      // Wait 4000us, retransmit up to 15 times on auto-ack failures
            WriteRegisterByte(0x05, 0x02);      // Use channel 2 frequency; change back second parameter to 0x08
            WriteRegisterByte(0x06, 0x0E);      // 2mbps, 0dBm RF output power
            WriteRegisterByte(0x07, 0x70);      // Clear all interrupts

            Thread.Sleep(10);

            //if (init)
            //{
            //    // Set pipe address
            //    byte[] RXAddressPipe0 = { 0x70, 0x70, 0x70 };  // 
            //    byte[] RXAddressPipe1 = { 0x71, 0x71, 0x71 };  // This will be the address of this computer
            //    WriteRegisterMultiByte(0x0A, RXAddressPipe0);  // Set RX address of pipe 0
            //    WriteRegisterMultiByte(0x0B, RXAddressPipe1);  // Set RX address of pipe 1
            //    WriteRegisterByte(0x0C, (byte)(RXAddressPipe0[2] + 2));  // Set RX address of pipe 2
            //    WriteRegisterByte(0x0D, (byte)(RXAddressPipe0[2] + 3));  // Set RX address of pipe 3
            //    WriteRegisterByte(0x0E, (byte)(RXAddressPipe0[2] + 4));  // Set RX address of pipe 4
            //    WriteRegisterByte(0x0F, (byte)(RXAddressPipe0[2] + 5));  // Set RX address of pipe 5
            //}

            byte[] RXAddressPipe0 = { 0x70, 0x70, 0x70 };
            //byte[] RXAddressPipe1 = { 0x71, 0x71, (byte)(0x70 + id)};  // This will be the address of this computer
            byte[] RXAddressPipe1 = { 0x77, 0x77, 0x77 };

            WriteRegisterMultiByte(0x0A, RXAddressPipe0);  // Set RX address of pipe 0
            WriteRegisterMultiByte(0x0B, RXAddressPipe1);  // Set RX address of pipe 1
            WriteRegisterByte(0x0C, (byte)(RXAddressPipe0[2] + 2));  // Set RX address of pipe 2
            WriteRegisterByte(0x0D, (byte)(RXAddressPipe0[2] + 3));  // Set RX address of pipe 3
            WriteRegisterByte(0x0E, (byte)(RXAddressPipe0[2] + 4));  // Set RX address of pipe 4
            WriteRegisterByte(0x0F, (byte)(RXAddressPipe0[2] + 5));  // Set RX address of pipe 5

            WriteRegisterByte(0x1C, 0x03);      // Enable Dynamic Payload Length on pipe x
            WriteRegisterByte(0x1D, 0x06);      // Enable payload length &  ack payload
             
            WriteRegisterByte(0x00, 0x3F);      // Power UP, RX data ready interrupt enabled, 1-byte CRC, PRX

            SendSingleByteCommand(0xE1);        // Flush TX
            SendSingleByteCommand(0xE2);        // Flush RX

            Thread.Sleep(10);

            AssertCE();
        }

        public static void SetRFRX(byte id)
        {
            stopMonitoring = true;

            ConfigureRFRX(id);

            if (MonitoringThread != null && MonitoringThread.IsAlive)
            {
                MonitoringThread.Abort();
                MonitoringThread.Join();
            }

            stopMonitoring = false;

            EventDispatchingThread = new Thread(new ThreadStart(ProcessPackets));
            EventDispatchingThread.Start();
            MonitoringThread = new Thread(new ThreadStart(MonitorIRQ));
            MonitoringThread.Start();
        }

        /// <summary>
        /// Writes data to the TX payload. Not implemented/test yet... use WriteAckPayload instead
        /// </summary>
        /// <param name="data">Data to place in TX payload buffer</param>
        private static void WritePayload(ref byte[] data)
        {
            if (Mode != RFOpMode.RFTXMode)
            {
                throw new InvalidOperationException("Attempting to transmit while not in TX mode.");
            }

            if (data.Length == 0 || data.Length > 32)
            {
                throw new ArgumentOutOfRangeException("Payload length to nRF must be between 1 and 32 bytes inclusive.");
            }

            WriteRegisterByte(0x07, 0x7E);  // Clear any interrupts
            WriteRegisterByte(0x00, 0x7A);  // Power up and be a transmitter
            SendSingleByteCommand(0xE1);    // Flush TX FIFO

            SendMultiByteCommand(0xA0, ref data);   // Send payload

            throw new NotImplementedException();

            //AssertCE();
            //System.Threading.Thread.Sleep(1);
            //DeassertCE();
        }

        /// <summary>
        /// Places data in the ACK payload buffer. This data will be piggybacked onto ACK packet responses.
        /// </summary>
        /// <param name="payload"></param>
        private static void WriteAckPayload(ref byte[] payload)
        {
            if (payload.Length == 0 || payload.Length > 32)
            {
                throw new ArgumentOutOfRangeException("Payload length to nRF must be between 1 and 32 bytes inclusive.");
            }

            SendMultiByteCommand(0xA9, ref payload);   // Write payload to pipe 1 to be piggybacked on next ACK packet
        }

        private static byte[] ReadPayload()
        {
            if (Mode != RFOpMode.RFRXMode)
            {
                throw new InvalidOperationException("Attempting to receive while not in RX mode");
            }

            byte[] cmd = new byte[33];
            byte[] spiOut = new byte[33];

            cmd[0] = 0x61; // Read RX buffer command
            transceiver.ReadWriteSPI(ref cmd, ref spiOut);

            byte[] data = new byte[32];
            Buffer.BlockCopy(spiOut, 1, data, 0, 32);
            return data;
        }

        private static void AssertCE()
        {
            transceiver.SetOutputGPIOPinValue(CEPin, 1);
        }

        private static void DeassertCE()
        {
            transceiver.SetOutputGPIOPinValue(CEPin, 0);
        }

        private static byte ReadStatusRegister()
        {
            return ReadRegisterByte(0x07);
        }

        public static void StopMonitoring()
        {
            if (MonitoringThread != null && MonitoringThread.IsAlive)
            {
                stopMonitoring = true;
                MonitoringThread.Join();
                EventDispatchingThread.Join();
                transceiver.End();
            }
        }

        private static void MonitorIRQ()
        {
            uint status, packetLen;

#if DEBUG
            profileSw.Start();
            long step1, step2, step3, step4;
            //long refTime = 0;
#endif
            while (true)
            {               
                /*  The RX_DR IRQ is asserted by a new packet arrival event. The procedure for handling this interrupt should
                    be: 1) read payload through SPI, 2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
                    payloads available in RX FIFO, 4) if there are more data in RX FIFO, repeat from step 1).
                */

                //if (profileSw.ElapsedMilliseconds - refTime < 1000)
                //    continue;

                //Console.WriteLine("{0}", (profileSw.ElapsedMilliseconds - refTime));
                //refTime = profileSw.ElapsedMilliseconds;

                if (IsIRQActive) 
                {
                    MissCount = 0;

                    do 
                    {
                        if (stopMonitoring)
                            break;
#if DEBUG
                        step1 = profileSw.ElapsedTicks;
#endif

                        packetLen = ReadPayloadWidth();
                        if (packetLen > 32)
                            SendSingleByteCommand(0xe2);            // Flush RX FIFO if value larger than 32...
                        
#if DEBUG
                        step2 = profileSw.ElapsedTicks - step1;
#endif

                        byte[] packet = ReadPayload();              //  ~0.8 ms

#if DEBUG
                        step3 = profileSw.ElapsedTicks - step2 - step1;
#endif

                        NewPackets.Enqueue(packet);
                        status = WriteRegisterByte(0x07, 0x70);     // Clear IRQ, ~0.2ms

#if DEBUG
                        step4 = profileSw.ElapsedTicks - step3 - step2 - step1;
                        //Console.WriteLine("{0} {1} {2} {3}", step1, step2, step3, step4);
#endif

                        //status = ReadStatusRegister();            // ~0.2 ms

                        byte[] TXPayload = null;

                        if (TXPayloads.TryDequeue(out TXPayload))
                        {
                            WriteAckPayload(ref TXPayload);
                        }

                    } 
                    while(((status & 0xE) != 0xE) && !stopMonitoring);
                }
                else
                {
                    MissCount++;

                    if (MissCount > 1024)   // We've not detected packets too many times... could the nRF's FIFOs be choked? Flush 'em.
                    {
                        //SendSingleByteCommand(0xe1);    // Flush TX FIFO
                        //SendSingleByteCommand(0xe2);    // Flush RX FIFO
                        MissCount = 0;
                    }
                }

                if (stopMonitoring)
                    break;
            }
        }

        private static void ProcessPackets()
        {
            byte[] packetToProcess = null;
            TJPacket dfPacket;

            while (stopMonitoring != true || NewPackets.Count > 0)
            {
                if (NewPackets.TryDequeue(out packetToProcess))
                {
                    dfPacket = new TJPacket(packetToProcess);
                    OnPacketReceived(new PacketReceivedEventArgs(dfPacket));
                }

                // Add a time delay of 1 millisecond per iteration to reduce CPU usage.
                //Thread.Sleep(1);
            }
        }

        protected static void OnPacketReceived(PacketReceivedEventArgs args)
        {
            PacketReceivedHandler handlers = PacketReceived;
            if (handlers != null)
                handlers(args);
        }

        public static void EnqueuePayload(byte[] payload)
        {
            TXPayloads.Enqueue(payload);
        }

        public static void ClearTXConcurrentQueue()
        {
            while (!TXPayloads.IsEmpty)
            {
                byte[] t = null; 
                bool b = TXPayloads.TryDequeue(out t);

                if (!b)
                    break;
            }
        }
    }
}
