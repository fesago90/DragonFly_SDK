using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TechJectDF;

/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 */

/*
 * This project is basic example of how the SDK can be used to receive/monitor Dragonfly packets
 * as well as transmitting packets to the Dragonfly.
 */

namespace ConsolePacketViewer
{
    class Program
    {
        static void Main(string[] args)
        {
            // First, initialize the connection
            TJDragonfly.Connect();

            // Set up a callback method that listens for the arrival of any new packet
            TJDragonfly.PacketReceived += TJDragonfly_PacketReceived;

            // Set up a call back method that handles the ctrl+c signal to exit the program
            Console.CancelKeyPress += Console_CancelKeyPress;
        }

        static void Console_CancelKeyPress(object sender, ConsoleCancelEventArgs e)
        {
            // Disconnect from RF controller and stop all monitoring/processing threads
            TJDragonfly.Disconnect();
        }

        static UInt16 count = 0;

        static void TJDragonfly_PacketReceived(TJPacket packet)
        {
            
            count += 1;

            // Print hexadecimal string representation of the raw packet
            string hex = BitConverter.ToString(packet.RawPacket).Replace("-", "");
            Console.WriteLine("[{0}] Got packet with ID = {1} and Seq# = {2}:\n{3}\n\n", packet.Timestamp, packet.PID, packet.Seq, hex);

            // Create a packet to send to the dragonfly
            byte[] txpacket = new byte[32];

            // Fill packet with data (we will be releasing a command specification in the future)
            txpacket[0] = (byte)(count & 0xff);
            txpacket[1] = (byte)((count >> 8 ) & 0xff);
            txpacket[2] = 0xaa;
            txpacket[3] = 0xbb;
            txpacket[4] = 0xcc;
            txpacket[5] = 0xdd;
            txpacket[6] = 0xee;
            txpacket[7] = 0xff;
            // txpacket[...] = ...;

            // Place packet in TX queue
            TJDragonfly.EnqueueTXPacket(txpacket);
        }

    }
}
