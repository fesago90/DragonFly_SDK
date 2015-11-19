/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 */

using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using TechJectDF;

namespace TechJectDF
{
    public class TJPacketExporter
    {

        /// <summary>
        /// Encapsulates the current capture progress
        /// </summary>
        public class CaptureProgressEventArgs : EventArgs
        {
            int numberOfPacketsProcessed, maxNumberOfPacketsToProcess;

            public CaptureProgressEventArgs( int numProcessed, int maxToProcess)
            {
                numberOfPacketsProcessed = numProcessed;
                maxNumberOfPacketsToProcess = maxToProcess;
            }

            public int NumberOfPacketsProcessed
            {
                get { return numberOfPacketsProcessed; }
            }
            public int MaxNumberOfPacketsToProcess
            {
                get { return maxNumberOfPacketsToProcess; }
            }
        }

        /// <summary>
        /// Describes the different formats in which the captures packets may be stored.
        /// </summary>
        public enum TJPacketExportFormat
        {
            /// <summary>
            /// String with comma separated values.
            /// </summary>
            CSV,
            /// <summary>
            /// Raw binary representation of the packets.
            /// </summary>
            Binary,
            /// <summary>
            /// Enables for custon packet formatting function by setting the CustomFormatter delegate.
            /// </summary>
            Custom
        };

        /// <summary>
        /// Describes a method that takes a raw packet, possibly parses it and returns the byte-representation of the formatted packet.
        /// </summary>
        /// <param name="packet">The packet</param>
        /// <returns>bytes representation of the formatted packet</returns>
        public delegate byte[] PacketFormatter(TJPacket packet);

        /// <summary>
        /// Reports the number of packets captured so far. This event fires when the packet buffer has been filled or the max number of packets have been captured.
        /// </summary>
        public event EventHandler<CaptureProgressEventArgs> CaptureProgressChanged;

        /// <summary>
        /// This property may be set to a custom method implementation following the signature defined by the PacketFormatter delegate.
        /// In order to use this custom formatter, the TJPacketExportFormat must be Custom when starting the capture.
        /// </summary>
        public PacketFormatter CustomFormatter
        { get; set; }

        const uint kBufferSize = 4096;
        int numProcessedPackets = 0;
        int maxNumberOfPackets;
        int numBytesInMemory = 0;
        LinkedList<TJPacket> bufferedPackets = new LinkedList<TJPacket>();
        TJPacketExportFormat packetFormat;
        bool isExternalStream = false;
        Stream outputStream;

        /// <summary>
        /// The constructor triggers a call to TJDragonfly.Connect() 
        /// </summary>
        public TJPacketExporter() 
        {
            TJDragonfly.DragonflyConnectionChanged += TJDragonfly_DragonflyConnectionChanged;
            TJDragonfly.Connect();
        }

        void TJDragonfly_DragonflyConnectionChanged()
        {
            if (outputStream != null) // Check whether we are currently Capturing packets
                StopRecording();
        }

        protected virtual void OnCaptureProgressChanged(CaptureProgressEventArgs args )
        {
            EventHandler<CaptureProgressEventArgs> handler = CaptureProgressChanged;
            if (handler != null)
                handler(this, args);
        }

        /// <summary>
        /// Listens for the Dragonfly's packets and forwards them to the a stream with the given packet format
        /// </summary>
        /// <param name="fmt">The format in which each packet is to be written</param>
        /// <param name="outputStream">The stream to write the packets to. This stream will not be disposed/closed automatically.</param>
        /// <param name="maxNumberOfPackets">The maximum number of packets to record</param>
        public void StartCapturing(Stream outputStream, TJPacketExportFormat fmt, int maxNumberOfPackets = 1000000)
        {
            if (maxNumberOfPackets <= 0)
                StopRecording();

            this.packetFormat = fmt;
            this.maxNumberOfPackets = maxNumberOfPackets;
            TJDragonfly.PacketReceived += TJDragonfly_PacketReceived;
        }

        /// <summary>
        /// Listens for the Dragonfly's packets and forwards them to the a file with the given packet format
        /// </summary>
        /// <param name="fmt">The format in which each packet is to be written</param>
        /// <param name="outputFilePath"></param>
        /// <param name="maxNumberOfPackets">The maximum number of packets to record</param>
        public void StartCapturing(string outputFilePath, TJPacketExportFormat fmt, int maxNumberOfPackets = 1000000)
        {
            isExternalStream = false;
            outputStream = new FileStream(outputFilePath, FileMode.Append, FileAccess.Write, FileShare.Read);
            StartCapturing(outputStream, fmt, maxNumberOfPackets);
        }

        /// <summary>
        /// Stops recording the Dragonfly's packets.
        /// </summary>
        public void StopRecording()
        {
            TJDragonfly.PacketReceived -= TJDragonfly_PacketReceived;

            if (isExternalStream == false && outputStream != null)
            {
                outputStream.Dispose();
                outputStream = null;
            }

            isExternalStream = true;
        }

        /// <summary>
        /// Called every time a packet has been received. Packet is placed in a buffer and progress events are fired
        /// when it is filled.
        /// </summary>
        /// <param name="packet">The received packet</param>
        void TJDragonfly_PacketReceived(TJPacket packet)
        {
            numProcessedPackets += 1;
            numBytesInMemory += packet.RawPacket.Length;
            bufferedPackets.AddLast(packet);

            if (numBytesInMemory > kBufferSize)
            {
                ExportPackets(bufferedPackets, outputStream, packetFormat);
                bufferedPackets.Clear();
                numBytesInMemory = 0;
                OnCaptureProgressChanged(new CaptureProgressEventArgs(numProcessedPackets, maxNumberOfPackets));
            }

            if (numProcessedPackets >= maxNumberOfPackets)
            {
                if (numBytesInMemory > 0)
                {
                    ExportPackets(bufferedPackets, outputStream, packetFormat);
                    OnCaptureProgressChanged(new CaptureProgressEventArgs(numProcessedPackets, maxNumberOfPackets)); 
                }

                StopRecording();
                numProcessedPackets = 0;
            }
            
        }

        /// <summary>
        /// Takes an IEnumerable of raw packets and exports them to a file in the given format.
        /// </summary>
        /// <param name="packets">The raw packets</param>
        /// <param name="outputFilename">Output filename</param>
        /// <param name="fmt">The format in which each packet is to be written</param>
        public static void ExportPackets(IEnumerable<TJPacket> packets, string outputFilePath, TJPacketExportFormat fmt)
        {
            FileStream outputStream = new FileStream(outputFilePath, FileMode.Append, FileAccess.Write, FileShare.Read);
            using (outputStream)
            {
                ExportPackets(packets, outputStream, fmt);
            }
        }

        /// <summary>
        /// Takes an IEnumerable of raw packets and exports them to a file in the given format.
        /// </summary>
        /// <param name="packets">The raw packets</param>
        /// <param name="outputStream">Output stream</param>
        /// <param name="fmt">The format in which each packet is to be written</param>
        public static void ExportPackets(IEnumerable<TJPacket> packets, Stream outputStream, TJPacketExportFormat fmt)
        {
            byte[] buffer = new byte[kBufferSize];
            byte[] formattedPacket;

            PacketFormatter formatter = ChoosePacketFormatter(fmt);

            foreach (TJPacket packet in packets)
            {
                formattedPacket = formatter(packet);
                outputStream.Write(formattedPacket, 0, formattedPacket.Length);
            }
        }

        static PacketFormatter ChoosePacketFormatter(TJPacketExportFormat fmt) 
        {
            PacketFormatter formatter = null;

            switch (fmt)
            {
                case TJPacketExportFormat.Binary:
                    formatter = PacketToBinary;
                    break;
                case TJPacketExportFormat.CSV:
                    formatter = PacketToCSV;
                    break;
                case TJPacketExportFormat.Custom:
                    break;
                default:
                    throw new ArgumentException("Invalid formatter type");
            }

            if (formatter == null) throw new InvalidOperationException("No packet formatter assigned");

            return formatter;
        }

        static byte[] PacketToCSV(TJPacket packet)
        {
            return packet.ToCSVBytes();
        }

        static byte[] PacketToBinary(TJPacket packet)
        {
            return packet.RawPacket;
        }
    }
}
