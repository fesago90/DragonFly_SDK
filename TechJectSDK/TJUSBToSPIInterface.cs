// for basic USB to SPI interface definition
// 
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TechJectDF
{
    public enum TJGPIOPins
    {
        TJGPIO_0 = 1,
        TJGPIO_1 = 2,
        TJGPIO_2 = 4,
        TJGPIO_3 = 8,
        TJGPIO_4 = 16,
        TJGPIO_5 = 32,
        TJGPIO_6 = 64,
        TJGPIO_7 = 128,
        TJGPIO_8 = 256,
    }

    public interface TJUSBToSPIInterface
    {
        bool Init();
        void SetGPIODirection(TJGPIOPins pinDirection);
        void SetOutputGPIOPinValue(TJGPIOPins pin, uint value);
        uint GetInputGPIOPinValue(TJGPIOPins pin);
        void WriteSPI(ref byte[] dataOut);
        void ReadWriteSPI(ref byte[] dataOut, ref byte[] dataIn);
        void End();
    }


}
