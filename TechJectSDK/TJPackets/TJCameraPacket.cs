/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Ian G, TechJect Inc.
 */

using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace TechJectDF
{
    public class TJCameraPacket : TJPacket
    {
        public UInt16 SegmentID;
        public UInt16[] SegmentPixels;

        public TJCameraPacket(byte[] rawCameraPacket)
            : base(rawCameraPacket)
        {
            if (BitConverter.IsLittleEndian != true)
                throw new NotImplementedException();

            UInt16[] segment = new UInt16[28];
            for (int i = 4; i < 32; i++) segment[i - 4] = rawCameraPacket[i];

            this.SegmentPixels = segment;
            this.SegmentID = BitConverter.ToUInt16(rawCameraPacket, 2);
        }
    }

    public class TJStartFramePacket : TJPacket
    {
        public UInt16 Rows;
        public UInt16 Cols;
        public UInt16 SegmentLength;

        public TJStartFramePacket(byte[] rawCameraPacket)
            : base(rawCameraPacket)
        {
            if (BitConverter.IsLittleEndian != true)
                throw new NotImplementedException();

            this.Cols = BitConverter.ToUInt16(rawCameraPacket, 2);
            this.Rows = BitConverter.ToUInt16(rawCameraPacket, 4);
            this.SegmentLength = BitConverter.ToUInt16(rawCameraPacket, 6);
        }
    }
}
