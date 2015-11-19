/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TechJectDF.TJCommands
{
    public class TJSetGainsCmd : TJCommand
    {
        public int ControlType
        { get; set; }

        public TJPIDControlGains ControlGains
        { get; set; }

        public TJPIDControlGains Roll
        { get; set; }

        public TJPIDControlGains Pitch
        { get; set; }

        public TJPIDControlGains Yaw
        { get; set; }

        // Overloading for the set gain functions
        public TJSetGainsCmd(int type, TJPIDControlGains control) : base(TJCommandID.SetGainsCmdID)
        {
            ControlType = type;
            ControlGains = control;
        }

        public TJSetGainsCmd(TJPIDControlGains roll, TJPIDControlGains pitch, TJPIDControlGains yaw)
            : base(TJCommandID.SetGainsCmdID)
        {
            Roll = roll; Pitch = pitch; Yaw = yaw;
        }

        public override byte[] ToRawPacket()
        {
            if (!BitConverter.IsLittleEndian)
                throw new NotImplementedException();

            byte[] raw = base.ToRawPacket();

            
            // New version
            if (ControlGains != null)
            {
                raw[2] = (byte)ControlType;

                raw[4] = (byte)(ControlGains.Kp & 0xFF);
                raw[5] = (byte)((ControlGains.Kp >> 8) & 0xFF);
                raw[6] = (byte)(ControlGains.Ki & 0xFF);
                raw[7] = (byte)((ControlGains.Ki >> 8) & 0xFF);
                raw[8] = (byte)(ControlGains.Kd & 0xFF);
                raw[9] = (byte)((ControlGains.Kd >> 8) & 0xFF);
            }

            // Old version
            //if (Roll != null)
            //{
            //    raw[2] |= (byte)TJControlType.RollControl;

            //    raw[4] = (byte)(Roll.Kp & 0xFF);
            //    raw[5] = (byte)((Roll.Kp >> 8) & 0xFF);
            //    raw[6] = (byte)(Roll.Ki & 0xFF);
            //    raw[7] = (byte)((Roll.Ki >> 8) & 0xFF);
            //    raw[8] = (byte)(Roll.Kd & 0xFF);
            //    raw[9] = (byte)((Roll.Kd >> 8) & 0xFF);

            //}

            //if (Pitch != null)
            //{
            //    raw[2] |= (byte)TJControlType.PitchControl;

            //    raw[10] = (byte)(Pitch.Kp & 0xFF);
            //    raw[11] = (byte)((Pitch.Kp >> 8) & 0xFF);
            //    raw[12] = (byte)(Pitch.Ki & 0xFF);
            //    raw[13] = (byte)((Pitch.Ki >> 8) & 0xFF);
            //    raw[14] = (byte)(Pitch.Kd & 0xFF);
            //    raw[15] = (byte)((Pitch.Kd >> 8) & 0xFF);
            //}

            //if (Yaw != null)
            //{
            //    raw[2] |= (byte)TJControlType.YawControl;

            //    raw[16] = (byte)(Yaw.Kp & 0xFF);
            //    raw[17] = (byte)((Yaw.Kp >> 8) & 0xFF);
            //    raw[18] = (byte)(Yaw.Ki & 0xFF);
            //    raw[19] = (byte)((Yaw.Ki >> 8) & 0xFF);
            //    raw[20] = (byte)(Yaw.Kd & 0xFF);
            //    raw[21] = (byte)((Yaw.Kd >> 8) & 0xFF);
            //}

            return raw;
        }
    }
}
