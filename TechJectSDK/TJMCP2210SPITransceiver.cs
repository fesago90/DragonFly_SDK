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
using MCP2210;

namespace TechJectDF
{
    public class TJMCP2210SPITransceiver : TJUSBToSPIInterface
    {
        private static object SPILock = new Object();

        const TJGPIOPins CS = TJGPIOPins.TJGPIO_0;
        const TJGPIOPins CE = TJGPIOPins.TJGPIO_1;
        const TJGPIOPins IRQ = TJGPIOPins.TJGPIO_2;

        const uint VID = 0x04D8;    // VID for Microchip Inc
        const uint PID = 0x00DE;    // PID for MCP2210
     
        MCP2210.DevIO device;

        uint GPIOOutputValues = 0;

        public bool Init()
        {
            device = new DevIO(VID, PID);
            bool connected = device.Settings.GetConnectionStatus();

            if (!connected)
            {
                Console.WriteLine("No devices connected");
                return false;
            }

            Console.WriteLine("{0} devices available", device.Special.GetDevCount());

            Console.WriteLine("Currently using device #{0}, {1}", 
                device.Special.GetSelectedDevNum(),
                device.Special.GetSelectedDevInfo());

            ConfigureMCP2210();

            return true;
        }

        public void SetGPIODirection(TJGPIOPins pinDirection)
        {
            device.Functions.SetGpioPinDir((ushort)pinDirection);
        }

        public void SetOutputGPIOPinValue(TJGPIOPins pin, uint value)
        {
            uint bitValue = (value << ((int)pin - 1));
            GPIOOutputValues &= ~value;
            GPIOOutputValues |= bitValue;
            device.Functions.SetGpioPinVal((ushort)GPIOOutputValues);
            //AardvarkApi.aa_gpio_set(handle, (byte)(GPIOOutputValues));
        }

        public uint GetInputGPIOPinValue(TJGPIOPins pin)
        {
            return ((uint)device.Functions.GetGpioPinVal() & (uint)pin) != 0 ? 1u : 0;
        }

        public void WriteSPI(ref byte[] dataOut)
        {
            if (dataOut.Length == 0)
                throw new ArgumentException("SPI write must have at least one byte of output data");

            lock (SPILock)
            {
                device.Settings.SetSpiTxferSize(DllConstants.CURRENT_SETTINGS_ONLY, (ushort)dataOut.Length);
                byte[] dataIn = new byte[dataOut.Length];
                device.Functions.TxferSpiData(dataOut, dataIn);
                //AardvarkApi.aa_spi_write(handle, (ushort)dataOut.Length, dataOut, (ushort)dataIn.Length, dataIn);
            }
        }

        public void ReadWriteSPI(ref byte[] dataOut, ref byte[] dataIn)
        {
            if (dataOut.Length == 0)
                throw new ArgumentException("SPI write must have at least one byte of output data");

            lock (SPILock)
            {
                device.Settings.SetSpiTxferSize(DllConstants.CURRENT_SETTINGS_ONLY, (ushort)dataOut.Length);
                device.Functions.TxferSpiData(dataOut, dataIn);
            }
        }

        public void End()
        {
            device.Functions.CancelSpiTxfer();
        }

        public void ConfigureMCP2210()
        {
            // GPIO0 is CS, GPIO1 is CE, and GPIO2 is IRQ
            byte[] gpioDesignation = {1, 0, 0, 0, 0, 0, 0, 0, 0};
            int res = device.Settings.SetGpioConfig(
                DllConstants.CURRENT_SETTINGS_ONLY, 
                gpioDesignation,                        // 0 = GPIO, 1 = use as CS, 2 = special fn as described in MCP2210 docs
                0xFFFF,                                      // default gpio values = low
                0xFFFF                                // Directions; 1 = input, 0 = output. IRQ is the only input
                );

            if (res != 0)
            {
                Console.WriteLine("Error setting GPIO settings. Err no {0}", res);
            }

            res = device.Settings.SetAllSpiSettings(
                DllConstants.CURRENT_SETTINGS_ONLY,
                3000000,                           // SPI rate
                0xFFFF,                             // Idle CS value
                0x0000,                             // Active CS value
                0,                                  // CS to data delay
                0,                                  // Data-to-data delay
                0,                                  // Data to CS delay
                1,                                  // Bytes per SPI transmission
                0                                   // SPI mode, (had to determine this using scope... can't find diagrams anywhere in microchip's docs)
                );

            if (res != 0)
            {
                Console.WriteLine("Error setting SPI settings. Err no {0}", res);
            }

            int bitrate = device.Settings.GetSpiBitRate(DllConstants.CURRENT_SETTINGS_ONLY);
            Console.WriteLine("SPI set at {0} kbps", bitrate/1000);
        }
    }
}
