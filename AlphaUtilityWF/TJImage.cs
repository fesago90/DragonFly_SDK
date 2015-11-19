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
using System.Diagnostics;
using System.Windows.Forms;
using System.Drawing;

using CustomExtensions;
using ZedGraph;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using TechJectDF;
using TechJectDF.TJCommands;

using System.Runtime.InteropServices;

namespace AlphaUtilityWF
{
    public class TJImage
    {
        public UInt16 Rows;
        public UInt16 Columns;
        UInt16 SegmentLength;
        UInt16 LastSegmentLength;
        public byte[] ImageArray;

        private int Min(int a, int b)
        {
            if (a < b)
                return a;
            else
                return b;
        }

        public TJImage(TJStartFramePacket newFrame)
        {
            this.Rows = newFrame.Rows;
            this.Columns = newFrame.Cols;
            this.SegmentLength = newFrame.SegmentLength;
            if (((int)this.Rows) * ((int)this.Columns) % ((int)this.SegmentLength) > 0) // the last packet won't be complete
            {
                this.LastSegmentLength = (UInt16)(((int)this.Rows) * ((int)this.Columns) % ((int)this.SegmentLength));
            }
            else // the last packet is same as previous packets
            {
                this.LastSegmentLength = this.SegmentLength;
            }
            this.LastSegmentLength = (UInt16)(((int)this.Rows) * ((int)this.Columns) % ((int)this.SegmentLength));
            this.ImageArray = new byte[(this.Rows) * (this.Columns)];
        }

        public void InsertSegment(TJCameraPacket Packet)
        {
            int MaxSegmentID = ((((int)this.Rows) * ((int)this.Columns) / ((int)this.SegmentLength))) + Min(LastSegmentLength, 1) - 1;
            /*
            int MaxSegmentID;
            if (((int)this.Rows) * ((int)this.Columns) % ((int)this.SegmentLength) > 0)
            {
                MaxSegmentID = ((((int)this.Rows) * ((int)this.Columns) / ((int)this.SegmentLength)));
            }
            else
            {
                MaxSegmentID = ((((int)this.Rows) * ((int)this.Columns) / ((int)this.SegmentLength))) - 1;
            }
            */
            if (Packet.SegmentID < MaxSegmentID)
            {
                for (int i = 0; i < this.SegmentLength; i++)
                {
                    this.ImageArray[Packet.SegmentID * this.SegmentLength + i] = (byte)Packet.SegmentPixels[i];
                }
            }
            else if (Packet.SegmentID == MaxSegmentID)
            {
                for (int i = 0; i < this.LastSegmentLength; i++)
                {
                    this.ImageArray[Packet.SegmentID * this.SegmentLength + i] = (byte)Packet.SegmentPixels[i];
                }
            }
        }
    }
}
