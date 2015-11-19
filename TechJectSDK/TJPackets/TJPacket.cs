/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 * Modified by Ian G, TechJect Inc.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TechJectDF
{
    public interface ITJExportablePacket
    {
        byte[] ToCSVBytes();
    }

    public enum TJPacketTypes
    {
        TJDFAnyType = -1,
        TJDFStateType = 0x00,
        TJDFState2Type = 0x01,
        TJDFIdentityType = 0x02,
        TJDFCameraType = 0x90,
        TJDFStartFrameType = 0x91
    }

    public class TJPacket : ITJExportablePacket
    {

        public readonly long Timestamp;

        public readonly byte[] RawPacket;

        public int PID
        { get { return RawPacket[0]; } }

        public int Seq
        { get { return RawPacket[1]; } }

        public TJPacket(byte[] rawPacket)
        {
            this.Timestamp = DateTime.Now.Ticks;
            this.RawPacket = rawPacket;
        }

        public virtual byte[] ToCSVBytes()
        {
            string csvString = String.Format("{0}, {1}, {2}\n", PID, Seq, BitConverter.ToString(RawPacket));
            return Encoding.ASCII.GetBytes(csvString);
        }
    }
}
