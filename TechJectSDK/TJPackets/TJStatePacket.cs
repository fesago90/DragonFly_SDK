
/*
 * This file is subject to the terms and conditions defined in
 * file 'License.txt', which is part of this source code package.
 *
 * Created by Felipe S, TechJect Inc.
 * Modified by Ian G, TechJect Inc.
 */

using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace TechJectDF
{
    public class GyroState
    {
        public readonly Int16 X, Y, Z;

        public GyroState(Int16 x, Int16 y, Int16 z)
        {
            X = x; Y = y; Z = z;
        }
    }

    public class AccelState
    {
        public readonly Int16 X, Y, Z;

        public AccelState(Int16 x, Int16 y, Int16 z)
        {
            X = x; Y = y; Z = z;
        }
    }

    public class BatteryState
    {
        public readonly Int16 LiBatt, Servo, Supply;

        public BatteryState(Int16 libatt, Int16 servo, Int16 supply)
        {
            LiBatt = libatt; Servo = servo; Supply = supply;
        }
    }

    public class MotorState
    {
        public readonly Int16 FrontLeft, FrontRight;//, RearLeft, RearRight;

        public MotorState(Int16 fl, Int16 fr)
        {
            FrontLeft = fl; FrontRight = fr; //RearLeft = rl; RearRight = rr;
        }

        //public MotorState(Int16 fl, Int16 fr, Int16 rl, Int16 rr)
        //{
        //    FrontLeft = fl; FrontRight = fr; RearLeft = rl; RearRight = rr;
        //}
    }

    public class CompassState
    {
        public readonly Int16 X, Y, Z;

        public CompassState(Int16 x, Int16 y, Int16 z)
        {
            X = x; Y = y; Z = z;
        }
    }

    public class PWMState
    {
        public readonly UInt16 FrontLeft, FrontRight, RearLeft, RearRight;

        public PWMState(UInt16 fl, UInt16 fr, UInt16 rl, UInt16 rr)
        {
            FrontLeft = fl; FrontRight = fr; RearLeft = rl; RearRight = rr;
        }
    }

    public class PosState
    {
        public readonly Int16 X, Y;

        public PosState(Int16 x, Int16 y)
        {
            X = x; Y = y;
        }
    }

    public class VelState
    {
        public readonly Int16 X, Y;

        public VelState(Int16 x, Int16 y)
        {
            X = x; Y = y;
        }
    }

    public class KFVSCF // defined only for the purpose of comparing kalman filter results with complimentary filter results
    {
        public readonly Int16 unbiasedGx, unbiasedGy, cfRoll, cfPitch;

        public KFVSCF(Int16 gx, Int16 gy, Int16 roll, Int16 pitch)
        {
            unbiasedGx = gx; unbiasedGy = gy;
            cfRoll = roll; cfPitch = pitch;
        }
    }

    public class PressureState
    {
        public readonly Int16 Pressure;

        public PressureState(Int16 p)
        {
            Pressure = p;
        }
    }

    public class HumidityState
    {
        public readonly Int16 Humidity;

        public HumidityState(Int16 h)
        {
            Humidity = h;
        }
    }

    public class TemperatureState
    {
        public readonly Int16 Temperature;

        public TemperatureState(Int16 t)
        {
            Temperature = t;
        }
    }

    public class GPSState
    {
        public readonly double Latitude, Longitude;

        public GPSState(byte Lat1, UInt32 Lat2, bool NS, byte Long1, UInt32 Long2, bool EW)
        {
            Latitude = (double)Lat1 + (double)Lat2 / 1000000;
            if (!NS) Latitude *= -1;

            Longitude = (double)Long1 + (double)Long2 / 1000000;
            if (!EW) Longitude *= -1;
        }
    }

    // Sonar altitude hold
    public class AltitudeState
    {
        public readonly byte Panic, Altitude;

        public AltitudeState(byte p, byte alt)
        {
            Panic = p; Altitude = alt;
        }
    }

    public class TJStatePacket : TJPacket
    {
        public GyroState Gyro;
        public AccelState Accel;
        public BatteryState Batt;
        public MotorState Motors;
        public PressureState Pressure;
        public TemperatureState Temperature;
        public HumidityState Humidity;
        public AltitudeState Altitide;

        public TJStatePacket(byte[] rawStatePacket)
            : base(rawStatePacket)
        {
            if (BitConverter.IsLittleEndian != true)
                throw new NotImplementedException();

            AccelState ast = new AccelState(
                BitConverter.ToInt16(rawStatePacket, 2),
                BitConverter.ToInt16(rawStatePacket, 4),
                BitConverter.ToInt16(rawStatePacket, 6)
                );

            GyroState gst = new GyroState(
                BitConverter.ToInt16(rawStatePacket, 8),
                BitConverter.ToInt16(rawStatePacket, 10),
                BitConverter.ToInt16(rawStatePacket, 12)
                );

            BatteryState bst = new BatteryState(
                BitConverter.ToInt16(rawStatePacket, 14),
                BitConverter.ToInt16(rawStatePacket, 16),
                BitConverter.ToInt16(rawStatePacket, 18)
                );

            MotorState mst = new MotorState(
                BitConverter.ToInt16(rawStatePacket, 20),
                BitConverter.ToInt16(rawStatePacket, 22)
                /*
                rawStatePacket[20],
                rawStatePacket[21],
                rawStatePacket[22],
                rawStatePacket[23]*/
                );

            PressureState pst = new PressureState(
                BitConverter.ToInt16(rawStatePacket, 24)
                );

            HumidityState hst = new HumidityState(
                BitConverter.ToInt16(rawStatePacket, 26)
            );

            TemperatureState tst = new TemperatureState(
                BitConverter.ToInt16(rawStatePacket, 28)
                );

            AltitudeState altst = new AltitudeState(
                rawStatePacket[30],
                rawStatePacket[31]
                );

            this.Batt = bst;
            this.Accel = ast;
            this.Gyro = gst;
            this.Motors = mst;

            this.Pressure = pst;
            this.Temperature = tst;
            this.Humidity = hst;
            this.Altitide = altst;
        }

        public override byte[] ToCSVBytes()
        {
            string csvString = String.Format(
                "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, ",// \n", // changed here to allow receive two state packet data
                Accel.X, Accel.Y, Accel.Z,
                Gyro.X, Gyro.Y, Gyro.Z,
                Batt.LiBatt, Batt.Servo, Batt.Supply, // vy, vx, 0
                Motors.FrontLeft, Motors.FrontRight, // ref pitch, ref roll
                Pressure.Pressure, // roll
                Humidity.Humidity, // pitch
                Temperature.Temperature, // yaw
                Altitide.Altitude
                );

            byte[] formattedPacket = Encoding.ASCII.GetBytes(csvString);

            return formattedPacket;
        }
    }

    public class TJState2Packet : TJPacket
    {
        public CompassState Compass;
        public PWMState PWMs;
        public PosState Pos;
        public VelState Vel;

        // for comparison between kalman filter results and complimentary filter results, the other place is in OfflineGraphsWindow.xaml.cs
        //public bool is_kfvscf = true; 
        public bool is_kfvscf = false;

        public KFVSCF Kfvscf;


        public TJState2Packet(byte[] rawStatePacket)
            : base(rawStatePacket)
        {
            if (BitConverter.IsLittleEndian != true)
                throw new NotImplementedException();

            CompassState cst = new CompassState(
                BitConverter.ToInt16(rawStatePacket, 2),
                BitConverter.ToInt16(rawStatePacket, 4),
                BitConverter.ToInt16(rawStatePacket, 6)
                );

            PWMState mst = new PWMState(
                BitConverter.ToUInt16(rawStatePacket, 8),
                BitConverter.ToUInt16(rawStatePacket, 10),
                BitConverter.ToUInt16(rawStatePacket, 12),
                BitConverter.ToUInt16(rawStatePacket, 14)
                );
            PosState pst = new PosState(
                BitConverter.ToInt16(rawStatePacket, 16),
                BitConverter.ToInt16(rawStatePacket, 18)
                );
            KFVSCF kfcf = new KFVSCF( // for comparison between kalman filter results and complimentary filter results
                BitConverter.ToInt16(rawStatePacket, 20),
                BitConverter.ToInt16(rawStatePacket, 22),
                BitConverter.ToInt16(rawStatePacket, 24),
                BitConverter.ToInt16(rawStatePacket, 26)
                );
            VelState vst = new VelState(
                BitConverter.ToInt16(rawStatePacket, 20),
                BitConverter.ToInt16(rawStatePacket, 22)
                );

            this.Compass = cst;
            this.PWMs = mst;
            this.Pos = pst;
            this.Kfvscf = kfcf; // for comparison between kalman filter results and complimentary filter results
            this.Vel = vst;

        }

        public override byte[] ToCSVBytes()
        {
            string csvString;

            if (is_kfvscf) // for comparison between kalman filter and complimentary filter results
            {
                csvString = String.Format(
                       "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}\n",
                       Compass.X, Compass.Y, Compass.Z,
                       PWMs.FrontLeft, PWMs.FrontRight, PWMs.RearLeft, PWMs.RearRight,
                       Pos.X, Pos.Y,
                       Kfvscf.unbiasedGx, Kfvscf.unbiasedGy, Kfvscf.cfRoll, Kfvscf.cfPitch
                   );
            }

            else
            {
                csvString = String.Format(
                    "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}\n",
                    Compass.X, Compass.Y, Compass.Z,
                    PWMs.FrontLeft, PWMs.FrontRight, PWMs.RearLeft, PWMs.RearRight,
                    Pos.X, Pos.Y,
                    Vel.X, Vel.Y
                );
            }

            byte[] formattedPacket = Encoding.ASCII.GetBytes(csvString);

            return formattedPacket;

        }
    }
}
