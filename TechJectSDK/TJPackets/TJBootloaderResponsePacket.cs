using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TechJectDF
{
    public class TJBootloaderResponsePacket : TJPacket
    {
        public int ConnectedAck;
        public int SendNextLine;
        public int SendLastLine;

        public TJBootloaderResponsePacket(byte[] rawBootloaderResponsePacket) : base(rawBootloaderResponsePacket)
        {
            if (BitConverter.IsLittleEndian != true)
                throw new NotImplementedException();

            this.SendNextLine = rawBootloaderResponsePacket[2];
            this.SendLastLine = rawBootloaderResponsePacket[3];
            this.ConnectedAck = rawBootloaderResponsePacket[4];
        }
    }
}
