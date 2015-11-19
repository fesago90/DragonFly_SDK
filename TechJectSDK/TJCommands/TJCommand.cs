/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 * Modified by Ian G, Techject Inc.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TechJectDF.TJCommands
{
    public interface ITJPacketizable
    {
        byte[] ToRawPacket();
    }

    public enum TJCommandID
    {
        ThrottleCmdID = 0x01,
        SetGainsCmdID = 0x02,
        TrimCmdID = 0x03,
        SetAddressCmdID = 0x04,
        AltHoldCmdID = 0x05,
        ArmingCmdID = 0x06,

        DFNomSatCmdID = 0x20,
        DFServoTrimCmdID = 0x21,
        DFPWMMinCmdID = 0x22,
        DFPWMMaxCmdID = 0x23,

        SetTogglesCmdID = 0x50,
        ConnectToBootloaderCmdID = 0x55,
        BootloaderDataCmdID = 0x56,

        ToggleFilterCmdID = 0x80,
        GetFrameCmdID = 0x90,
        EnableDisableCameraCmdID = 0x91,
        CameraSettingCmdID = 0x92,
        EnableDisablePX4CmdID = 0x93,
    }

    public class TJCommand : ITJPacketizable
    {
        public readonly TJCommandID CID;

        public int Seq
        { get; set; }

        public TJCommand(TJCommandID cid)
        {
            CID = cid;
        }

        public virtual byte[] ToRawPacket()
        {
            byte[] raw = new byte[32];
            raw[0] = (byte)CID;
            raw[1] = (byte)Seq;
            return raw;
        }
    }

    public class TJConnectToBootloaderCmd : TJCommand
    {
        public TJConnectToBootloaderCmd()
            : base(TJCommandID.ConnectToBootloaderCmdID)
        {
        }
    }

    public class TJBootloaderDataCmd : TJCommand
    {
        public byte[] Data;
        public TJBootloaderDataCmd(string DataLine)
            : base(TJCommandID.BootloaderDataCmdID)
        {
            Data = new byte[30]; // Define length of data

            DataLine = DataLine + '\r'; // re-add carriage return
            char[] DataChar = DataLine.ToCharArray(); // convert string to char
            int i = 1;
            for (i = 1; i < DataLine.Length / 2; i++)
            {
                Data[i] = (byte)(ConvertASCIIToHex(DataChar[2 * (i - 1) + 1], 'M') + ConvertASCIIToHex(DataChar[2 * (i - 1) + 2], 'L'));
            }
            if (DataLine.Length % 2 == 1)
            {
                Data[i] = ConvertASCIIToHex(DataChar[2 * (i - 1) + 1], 'M');
                i++;
            }
            Data[0] = 58; // ':', 0x3A
            Data[i] = 13; // '\r', carriage return
            Data[0]++;
            Data[0]--;
        }

        public override byte[] ToRawPacket()
        {
            if (!BitConverter.IsLittleEndian)
                throw new NotImplementedException();

            byte[] raw = base.ToRawPacket();

            for (int i = 0; i < Data.Length; i++)
            {
                raw[i + 2] = Data[i];
            }

            return raw;
        }

        public byte ConvertASCIIToHex(char ASCII, char SB)
        {
            byte output = 0;
            if (ASCII == '0') output = 0;
            if (ASCII == '1') output = 1;
            if (ASCII == '2') output = 2;
            if (ASCII == '3') output = 3;
            if (ASCII == '4') output = 4;
            if (ASCII == '5') output = 5;
            if (ASCII == '6') output = 6;
            if (ASCII == '7') output = 7;
            if (ASCII == '8') output = 8;
            if (ASCII == '9') output = 9;
            if (ASCII == 'a') output = 10;
            if (ASCII == 'b') output = 11;
            if (ASCII == 'c') output = 12;
            if (ASCII == 'd') output = 13;
            if (ASCII == 'e') output = 14;
            if (ASCII == 'f') output = 15;
            if (ASCII == 'F') output = 15;

            if ((SB == 'M') || (SB == 'm'))
            {
                output <<= 4;
            }

            return output;
        }
    }
}

