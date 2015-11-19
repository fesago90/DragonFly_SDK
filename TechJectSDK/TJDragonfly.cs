/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 * Modified by Ian G, TechJect Inc.
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.Serialization.Formatters.Binary;
using TechJectDF.TJCommands;
using System.IO;


namespace TechJectDF
{
    public enum TJControlType 
    {
        // Old protocol
        RollControl = 1,
        PitchControl = 2,
        YawControl = 4

        // New protocol
        //PositionRoll = 1,
        //PositionPitch = 2,
        //PositionYaw = 3,

        //AngleRoll = 4,
        //AnglePitch = 5,
        //AngleYaw = 6,

        //AngularRateRoll = 7,
        //AngularRatePitch = 8,
        //AngularRateYaw = 9,
    }

    public class TJPIDControlGains
    {
        public Int16 Kp { get; set; }
        public Int16 Ki { get; set; }
        public Int16 Kd { get; set; }

        public TJPIDControlGains()
        { Kp = 0; Ki = 0; Kd = 0; }

        public TJPIDControlGains(Int16 p, Int16 i, Int16 d)
        {
            Kp = p; Ki = i; Kd = d;
        }
    }

    public class TJDragonfly
    {
        static Stopwatch sw;
        static object connectionLock = new object();

        /// <summary>
        /// Reflects the status of the connection with the RF controller
        /// </summary>
        public static bool IsConnected
        {
            get { return isConnected; }
        }

        static bool isConnected     = false;
        static long lastTime        = 0;
        static int lastSeqNum       = 0;
        public static int nRFController    = 1; // 0 SPI-Click; 1: Aardvark
        public static byte nRFChID  = 0;

        /// <summary>
        /// This event is triggered whenever a TJStatePacket arrives.
        /// </summary>
        public static event Action<TJStatePacket> DragonflyStateUpdate;

        /// <summary>
        /// This event is triggered whenever a TJState2Packet arrives
        /// </summary>
        public static event Action<TJState2Packet> DragonflyState2Update;

        /// <summary>
        /// This event is triggered whenever a TJConnectedToBootloaderPacket arrives.
        /// </summary>
        public static event Action<TJBootloaderResponsePacket> DragonflyBootloaderUpdate;

        /// <summary>
        /// This event is triggered whenever a TJCameraPacket arrives.
        /// </summary>
        public static event Action<TJCameraPacket> DragonflyCameraUpdate;

        /// <summary>
        /// This event is triggered whenever a TJStartFramePacket arrives.
        /// </summary>
        public static event Action<TJStartFramePacket> DragonflyStartFrame;

        /// <summary>
        /// This event is triggered whenever a TJEndFramePacket arrives
        /// </summary>
        public static event Action DragonflyEndFrame;

        /// <summary>
        /// This event is triggered on the arrival of any packet.
        /// </summary>
        public static event Action<TJPacket> PacketReceived;

        /// <summary>
        /// This event is triggered when the RF controller and digital comm device have been successfully initialized
        /// or when the packet monitoring/processing threads have been terminated after a call to Disconnect().
        /// </summary>
        public static event Action DragonflyConnectionChanged;

        /// <summary>
        /// Initializes the RF and digital communications controller, then starts monitoring for packets.
        /// </summary>
        /// <returns>True if the controllers were successfully initialized, false otherwise.</returns>
        public static bool Connect()
        {
            if (isConnected)
                return true;

            bool rfConnectionResult;

            lock (connectionLock)
            {
                if (isConnected)
                    return true;

                // Whenever the RF controller receives data, it will be forwarded to the ProcessNewPacket function
                rfConnectionResult = TJnRFController.Init(nRFController, nRFChID, new TJnRFController.PacketReceivedHandler(ProcessNewPacket));

                sw = Stopwatch.StartNew();

                if (rfConnectionResult)
                {
                    Console.WriteLine("Monitoring for packets...");
                    isConnected = true;
                }
                else
                {
                    Console.WriteLine("Unable to set up connection");
                    isConnected = false;
                }

            }

            if (DragonflyConnectionChanged != null)
                DragonflyConnectionChanged();

            return isConnected;
        }
        
        /// <summary>
        /// Stops monitoring/processing packets.
        /// </summary>
        public static void Disconnect()
        {
            TJnRFController.StopMonitoring();
            Console.WriteLine("Disconnected");
            isConnected = false;
            if (DragonflyConnectionChanged != null)
                DragonflyConnectionChanged();
        }

        public static void setRFController(int cont)
        {
            nRFController = cont;
        }
        
        public static int getRFController()
        {
            return nRFController;
        }

        static int missed = 0;
        static int received = 0;

        /// <summary>
        /// This function is called by the RF controller when data has been received. Here we identify what type of packet
        /// it is and instantiate an appropriate packet class to hold the data. The packet instance is then forwarded to
        /// all listeners of the PacketReceived event
        /// </summary>
        /// <param name="args"></param>
        private static void ProcessNewPacket(PacketReceivedEventArgs args)
        {
            int now = (int)sw.ElapsedTicks;

            TJPacket packet = args.Packet;

            switch (packet.PID)
            {
                case 0x00:
                    packet = new TJStatePacket(packet.RawPacket);
                    if (DragonflyStateUpdate != null)
                        DragonflyStateUpdate(packet as TJStatePacket);
                    break;

                case 0x01:
                    packet = new TJState2Packet(packet.RawPacket);
                    if (DragonflyState2Update != null)
                        DragonflyState2Update(packet as TJState2Packet);
                    break;

                case 0x02:
                    // Scale factors packets
                    break;

                case 0x3A:
                    packet = new TJBootloaderResponsePacket(packet.RawPacket);
                    if (DragonflyBootloaderUpdate != null)
                        DragonflyBootloaderUpdate(packet as TJBootloaderResponsePacket);
                    break;

                case 0x90:
                    packet = new TJCameraPacket(packet.RawPacket);
                    if (DragonflyCameraUpdate != null)
                        DragonflyCameraUpdate(packet as TJCameraPacket);
                    break;

                case 0x91:
                    packet = new TJStartFramePacket(packet.RawPacket);
                    if (DragonflyStartFrame != null)
                        DragonflyStartFrame(packet as TJStartFramePacket);
                    break;

                case 0x92:
                    if (DragonflyEndFrame != null)
                        DragonflyEndFrame();
                    break;

                default: break;
            }

            if (PacketReceived != null)
                PacketReceived(packet);

            // The following code is just to keep track of how many packets have been lost.

            int dif = 2;

            if (packet.Seq < lastSeqNum)
            {
                int seqdif = packet.Seq + 256 - lastSeqNum;
                if (seqdif != dif)
                {
                    missed += seqdif / dif;
                }
            }
            else if (packet.Seq - lastSeqNum != dif)
            {
                missed += packet.Seq - lastSeqNum;
            }

            received++;

            if (received == 100)
            {
                Console.WriteLine("s/Packet: {0:0.00000}, missed: {1}", ((now - lastTime) / (double)received) / Stopwatch.Frequency, missed);
                lastTime = now;
                missed = 0;
                received = 0;
            }

            lastSeqNum = packet.Seq;
        }

        /// <summary>
        /// Places a raw TX packet into the queue to be transmitted by the RF controller.
        /// </summary>
        /// <param name="packet"></param>
        public static void EnqueueTXPacket(byte[] packet)
        {
            if (IsConnected)
                TJnRFController.EnqueuePayload(packet);
        }

        /// <summary>
        /// Places a command packet into the tx queue.
        /// </summary>
        /// <param name="cmd"></param>
        public static void EnqueueCommand(TJCommand cmd)
        {
            EnqueueTXPacket(cmd.ToRawPacket());
        }

        /// <summary>
        /// Updates the PID gains for each PID control in the MAV. Parameters may be left null if those are not to be updated.
        /// </summary>
        /// <param name="control_index">Position 0, Angle 1, Angular Rate 2</param>
        /// <param name="direction_index">Roll 0, Pitch 1, Yaw 2</param>
        public static void SetGains(int control_index, int direction_index, TJPIDControlGains control)
        {
            int type = control_index * 3 + direction_index + 1;

            TJSetGainsCmd cmd = new TJSetGainsCmd(type, control);
            EnqueueCommand(cmd);
        }

        /// <summary>
        /// Updates the PID gains for each PID control in the MAV. Parameters may be left null if those are not to be updated.
        /// </summary>
        /// <param name="roll">Roll PID gains,</param>
        /// <param name="pitch">Pitch PID gains</param>
        /// <param name="yaw">Yaw PID gains</param>
        public static void SetGains(TJPIDControlGains roll, TJPIDControlGains pitch, TJPIDControlGains yaw)
        {
            TJSetGainsCmd cmd = new TJSetGainsCmd(roll, pitch, yaw);
            EnqueueCommand(cmd);
        }
    }
}

