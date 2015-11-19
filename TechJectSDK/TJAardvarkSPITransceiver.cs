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
using TotalPhase;
using System.IO.Ports;

namespace TechJectDF
{


    class TJAardvarkSPITransceiver : TJUSBToSPIInterface 
    {
        private static object SPILock = new Object();


        public const int SPI_BITRATE = 3000;
        private const AardvarkGpioBits CEPin = AardvarkGpioBits.AA_GPIO_SDA;
        private const AardvarkGpioBits IRQPin = AardvarkGpioBits.AA_GPIO_SCL;

        private static int  handle;
        private static uint GPIOOutputValues = 0;

        public bool Init()
        {
            if (ConnectToAardvark())
            {
                ConfigureAardvark();
                return true;
            }

            return false;
        }

        public void End()
        {
            DisconnectAardvark();
        }

        // In mask, 1 = input, 0 = output
        public void SetGPIODirection(TJGPIOPins pinDirection)
        {
            // For Aardvark, 1 = output, 0 = input
            AardvarkApi.aa_gpio_direction(handle, (byte)(~pinDirection));
        }

        public void SetOutputGPIOPinValue(TJGPIOPins pin, uint value)
        {
            uint bitValue = (value << ((int)pin - 1));
            GPIOOutputValues &= ~value;
            GPIOOutputValues |= bitValue;
            AardvarkApi.aa_gpio_set(handle, (byte)(GPIOOutputValues));
        }

        // Returns true if pin is high, false if pin is low
        public uint GetInputGPIOPinValue(TJGPIOPins pin)
        {
            return ((uint)AardvarkApi.aa_gpio_get(handle) & (uint)pin) != 0 ? 1u : 0;
        }


        public void WriteSPI(ref byte[] dataOut) 
        {
            if (dataOut.Length == 0)
                throw new ArgumentException("SPI write must have at least one byte of output data");

            lock(SPILock)
            {
                CheckHandle();

                byte[] dataIn = new byte[1];

                AardvarkApi.aa_spi_write(handle, (ushort)dataOut.Length, dataOut, (ushort)dataIn.Length, dataIn);
            }
        }

        public void ReadWriteSPI(ref byte[] dataOut, ref byte[] dataIn)
        {
            if (dataOut.Length == 0)
                throw new ArgumentException("SPI write must have at least one byte of output data");

            lock (SPILock)
            {
                CheckHandle();
                AardvarkApi.aa_spi_write(handle, (ushort)dataOut.Length, dataOut, (ushort)dataIn.Length, dataIn);
            }
        }

        private void CheckHandle()
        {
            if (handle <= 0) {
                throw new InvalidOperationException("Not connected to SPI device");
            }
        }

        private bool ConnectToAardvark()
        {
            ushort[] ports = new ushort[16];
            int numElem = 16;
            int i;
            bool found = false;

            // Find all the attached devices
            int count = AardvarkApi.aa_find_devices(16, ports);

            if (count > numElem) count = numElem;

            for (i = 0; i < count; ++i)
            {
                // Determine if the device is in-use
                if ((ports[i] & AardvarkApi.AA_PORT_NOT_FREE) == 0)
                {
                    found = true;
                    break;
                }
            }

            if (found == false)
            {
                Console.WriteLine("No Aardvark devices found");
                return false;
            }

            handle = AardvarkApi.aa_open(ports[i]);

            if (handle <= 0)
            {
                Console.WriteLine("Unable to open Aardvark device on port {0}", ports[i]);
                Console.WriteLine("error: {0}", AardvarkApi.aa_status_string(handle));
                return false;
            }

            return true;
        }

        private void DisconnectAardvark()
        {
            AardvarkApi.aa_close(handle);
            handle = -1;
        }

        private void ConfigureAardvark()
        {
            // Ensure that the SPI subsystem is enabled
            AardvarkApi.aa_configure(handle, AardvarkConfig.AA_CONFIG_SPI_GPIO);

            // Enable the Aardvark adapter's power pins.
            // This command is only effective on v2.0 hardware or greater.
            // The power pins on the v1.02 hardware are not enabled by default.
            AardvarkApi.aa_target_power(handle, AardvarkApi.AA_TARGET_POWER_BOTH);

            // Turn off the external I2C line pullups
            AardvarkApi.aa_i2c_pullup(handle, AardvarkApi.AA_I2C_PULLUP_NONE);

            // Ensure master mode
            AardvarkApi.aa_spi_slave_disable(handle);

            // Set direction for IRQ, and CE pins
            // CE  = SDA should be an output
            // IRQ = SCL should be an input
            // By default, GPIO pins are inputs. Writing 1 sets it to an output
            AardvarkApi.aa_gpio_direction(handle, 0);

            // Setup the clock phase
            AardvarkApi.aa_spi_configure(handle,
                AardvarkSpiPolarity.AA_SPI_POL_RISING_FALLING,
                AardvarkSpiPhase.AA_SPI_PHASE_SAMPLE_SETUP,
                AardvarkSpiBitorder.AA_SPI_BITORDER_MSB);

            // Ensure chip select is active low
            AardvarkApi.aa_spi_master_ss_polarity(handle, AardvarkSpiSSPolarity.AA_SPI_SS_ACTIVE_LOW);


            // Setup the bitrate
            int bitrate = AardvarkApi.aa_spi_bitrate(handle, SPI_BITRATE);
            Console.WriteLine("SPI bitrate set to {0} kbps", bitrate);
        }

    }
}
