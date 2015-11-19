using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Forms.Integration;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using TechJectDF;
using ZedGraph;

namespace AlphaUtilityWF
{
    /// <summary>
    /// Interaction logic for OfflineGraphsWindow.xaml
    /// </summary>
    public partial class OfflineGraphsWindow : Window
    {
        WindowsFormsHost winFormHost;
        ZedGraphControl graph = new ZedGraphControl();

        float kAccel = MainWindow.kAccelScaleFactor;
        float kGyro = MainWindow.kGyroScaleFactor;
        float kOFC = MainWindow.kOFCScaleFactor;
        float kRefAngle = MainWindow.kRefAngleScalerFactor;
        float kAttitude = MainWindow.kAttitudeScalerFactor;
        float kPWM = MainWindow.kPWMScalerFactor; // for dc motor quad
        //float kPWM = MainWindow.kPWMBLDCScalerFactor; // for bldc motor quad
        //float kServoMin = MainWindow.kServoMinScalerFactor; // for bldc motor quad
        float kPos = MainWindow.kPosScalerFactor;
        float kVel = MainWindow.kPosScalerFactor;

        // for comparison between kalman filter results and complimentary filter results, the other place is in TJStatePacket.cs
        //bool is_kfvscf = true;
        bool is_kfvscf = false;

        List<List<int>> data;

        public OfflineGraphsWindow(List<List<int>> external_data)
        {
            InitializeComponent();

            data = new List<List<int>>();
            data = external_data;

            // Custom setup
            winFormHost = wfhOffline;
            winFormHost.Child = graph;
            cbbItem.SelectedIndex = 0;
            setupGraphs();
            UpdateGraphPane();
            drawGraphs();
        }

        void setupGraphs()
        {
            GraphPane pane = graph.GraphPane;

            pane.Title.Text = "Graphs";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "g";

            pane.XAxis.Scale.FontSpec.Size = 10;
            pane.XAxis.Title.FontSpec.Size = 10;
            pane.YAxis.Scale.FontSpec.Size = 10;
            pane.YAxis.Title.FontSpec.Size = 10;
            pane.Legend.FontSpec.Size = 10;
            pane.Title.FontSpec.Size = 10;
        }

        void drawGraphs()
        {
            int packet_state_num;

            if (!is_kfvscf)
                packet_state_num = 26; // need to change here if increase/decrease the size of each line (adding/subtracting state)
            else
                packet_state_num = 30; // for comparison between kalman filter results and complimentary filter results

            if (data.Count <= 0)
                return;

            CurveList curveList = graph.GraphPane.CurveList;

            double t = 0;
            foreach (var d in data)
            {
                if (d.Count != packet_state_num)
                    continue;

                List<PointPair> points = new List<PointPair>();

                switch (cbbItem.SelectedIndex)
                {
                    case 0:
                        {
                            // Accelerometer
                            points.Add(new PointPair(t, d[0] * kAccel));
                            points.Add(new PointPair(t, d[1] * kAccel));
                            points.Add(new PointPair(t, d[2] * kAccel));
                            break;
                        }
                    case 1:
                        {
                            // Gyro
                            points.Add(new PointPair(t, d[3] * kGyro));
                            points.Add(new PointPair(t, d[4] * kGyro));
                            points.Add(new PointPair(t, d[5] * kGyro));
                            break;
                        }
                    case 2:
                        {
                            if (!is_kfvscf)
                            {
                                // Optical Flow
                                points.Add(new PointPair(t, d[6] * kOFC)); // vx
                                points.Add(new PointPair(t, d[7] * kOFC)); // vy
                                // the third one is not displayed
                                break;
                            }
                            else // for comparison between kalman filter results and complimentary filter results
                            { // biased gyro x vs. unbiased gyro x
                                points.Add(new PointPair(t, d[3] * kGyro)); // biased
                                points.Add(new PointPair(t, d[24] * kGyro)); // unbiased
                                break;
                            }
                        }
                    case 3:
                        {
                            if (!is_kfvscf)
                            {
                                // Ref Angles
                                points.Add(new PointPair(t, d[9] * kRefAngle));  // ref pitch
                                points.Add(new PointPair(t, d[10] * kRefAngle)); // ref roll
                                break;
                            }
                            else // for comparison between kalman filter results and complimentary filter results
                            { // biased gyro y vs. unbiased gyro y
                                points.Add(new PointPair(t, d[4] * kGyro)); // biased
                                points.Add(new PointPair(t, d[25] * kGyro)); // unbiased
                                break;
                            }
                        }
                    case 4:
                        {
                            if (!is_kfvscf)
                            {
                                // Attitude - roll pitch yaw
                                points.Add(new PointPair(t, d[11] * kAttitude)); // roll
                                points.Add(new PointPair(t, d[12] * kAttitude)); // pitch
                                points.Add(new PointPair(t, d[13] * kAttitude)); // yaw
                                break;
                            }
                            else // for comparison between kalman filter results and complimentary filter results
                            {
                                points.Add(new PointPair(t, d[26] * kAttitude)); // complimentary filter roll
                                points.Add(new PointPair(t, d[11] * kAttitude)); // kalman filter roll
                                break;
                            }
                        }
                    case 5:
                        {
                            if (!is_kfvscf)
                            {
                                // Gyro x vs OFC x
                                points.Add(new PointPair(t, d[3] * kGyro));
                                points.Add(new PointPair(t, d[6] * kOFC));
                                break;
                            }
                            else // for comparison between kalman filter results and complimentary filter results
                            {
                                points.Add(new PointPair(t, d[27] * kAttitude)); // complimentary filter pitch
                                points.Add(new PointPair(t, d[12] * kAttitude)); // kalman filter pitch
                                break;
                            }
                        }
                    case 6:
                        {
                            // Gyro y vs OFC y
                            points.Add(new PointPair(t, d[4] * kGyro));
                            points.Add(new PointPair(t, d[7] * kOFC * (-1)));
                            break;
                        }
                    case 7:
                        {
                            // Sonar reading
                            points.Add(new PointPair(t, d[14]));
                            break;
                        }
                    case 8:
                        {
                            // Magnetometer
                            points.Add(new PointPair(t, d[15] - 10000));
                            points.Add(new PointPair(t, d[16] - 10000));
                            points.Add(new PointPair(t, d[17] - 10000));
                            break;
                        }
                    case 9:
                        {
                            // Motor PWMs for dc motor quad
                            points.Add(new PointPair(t, d[18] * kPWM));
                            points.Add(new PointPair(t, d[19] * kPWM));
                            points.Add(new PointPair(t, d[20] * kPWM));
                            points.Add(new PointPair(t, d[21] * kPWM));
                            break;
                            //// Motor PWMs for bldc motor quad
                            //points.Add(new PointPair(t, (d[18] - kServoMin) * kPWM));
                            //points.Add(new PointPair(t, (d[19] - kServoMin) * kPWM));
                            //points.Add(new PointPair(t, (d[20] - kServoMin) * kPWM));
                            //points.Add(new PointPair(t, (d[21] - kServoMin) * kPWM));
                            //break;
                        }
                    case 10:
                        { 
                            // Position, x and y
                            points.Add(new PointPair(t, d[22] * kPos));
                            points.Add(new PointPair(t, d[23] * kPos));
                            break;
                        }
                    case 11:
                        {
                            // Velocity, x and y
                            points.Add(new PointPair(t, d[24] * kVel));
                            points.Add(new PointPair(t, d[25] * kVel));
                            break;
                        }
                    default:
                        break;
                }

                for (int i = 0; i < curveList.Count; i++)
                {
                    IPointListEdit ip = (IPointListEdit)graph.GraphPane.CurveList[i].Points;
                    ip.Add(points[i]);
                }

                t = t + 0.01;
            }

            graph.AxisChange();
            graph.Refresh();
        }

        void UpdateGraphPane()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.Black };
            GraphPane pane = graph.GraphPane;
            pane.CurveList.Clear();

            switch (cbbItem.SelectedIndex)
            {
                case 0:
                    {
                        LineItem curve = pane.AddCurve("Accel X", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Accel Y", new PointPairList(), colors[1]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Accel Z", new PointPairList(), colors[2]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        pane.Title.Text = "Accel";
                        pane.YAxis.Title.Text = "g";
                        break;
                    }
                case 1:
                    {
                        LineItem curve = pane.AddCurve("Gyro X", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Gyro Y", new PointPairList(), colors[1]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Gyro Z", new PointPairList(), colors[2]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        pane.Title.Text = "Gyro";
                        pane.YAxis.Title.Text = "dsp";
                        break;
                    }
                case 2:
                    {
                        if (!is_kfvscf)
                        {
                            LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "OFC";
                            pane.YAxis.Title.Text = "dps";
                            break;
                        }
                        else  // for comparison between kalman filter results and complimentary filter results
                        {
                            LineItem curve = pane.AddCurve("Biased Gx", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Uniased Gx", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "Biased vs. unbiased Gx";
                            pane.YAxis.Title.Text = "dps";
                            break;
                        }
                    }
                case 3:
                    {
                        if (!is_kfvscf)
                        {
                            LineItem curve = pane.AddCurve("Ref Pitch", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Ref Roll", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            //curve = pane.AddCurve("Motor RL", new PointPairList(), colors[2]);
                            //curve.Symbol.Type = SymbolType.None;
                            //curve.Line.IsOptimizedDraw = true;

                            //curve = pane.AddCurve("Motor RR", new PointPairList(), colors[3]);
                            //curve.Symbol.Type = SymbolType.None;
                            //curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "Reference Angles";
                            pane.YAxis.Title.Text = "deg";
                            break;
                        }
                        else  // for comparison between kalman filter results and complimentary filter results
                        {
                            LineItem curve = pane.AddCurve("Biased Gy", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Uniased Gy", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "Biased vs. unbiased Gy";
                            pane.YAxis.Title.Text = "dps";
                            break;
                        }
                    }
                case 4:
                    {
                        if (!is_kfvscf)
                        {
                            LineItem curve = pane.AddCurve("Roll", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Pitch", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Yaw", new PointPairList(), colors[2]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "Attitude";
                            pane.YAxis.Title.Text = "deg";
                            break;
                        }
                        else  // for comparison between kalman filter results and complimentary filter results
                        {
                            LineItem curve = pane.AddCurve("Roll (CF)", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Roll (KF)", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "CF vs. KF Roll";
                            pane.YAxis.Title.Text = "deg";
                            break;
                        }
                    }
                case 5:
                    {
                        if (!is_kfvscf)
                        {
                            LineItem curve = pane.AddCurve("Gyro x", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("OFC x", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "Gyro x vs OFC x -- Pitch";
                            pane.YAxis.Title.Text = "dsp";
                            break;
                        }
                        else  // for comparison between kalman filter results and complimentary filter results
                        {
                            LineItem curve = pane.AddCurve("Pitch (CF)", new PointPairList(), colors[0]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            curve = pane.AddCurve("Pitch (KF)", new PointPairList(), colors[1]);
                            curve.Symbol.Type = SymbolType.None;
                            curve.Line.IsOptimizedDraw = true;

                            pane.Title.Text = "CF vs. KF Pitch";
                            pane.YAxis.Title.Text = "deg";
                            break;
                        }
                    }
                case 6:
                    {
                        LineItem curve = pane.AddCurve("Gyro y", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("OFC y", new PointPairList(), colors[1]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        pane.Title.Text = "Gyro y vs OFC y -- Roll";
                        pane.YAxis.Title.Text = "dsp";
                        break;
                    }
                case 7:
                    {
                        LineItem curve = pane.AddCurve("Sonar reading", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        pane.Title.Text = "Sonar reading";
                        pane.YAxis.Title.Text = "cm";
                        break;
                    }
                case 8:
                    {
                        LineItem curve = pane.AddCurve("Compass X", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Compass Y", new PointPairList(), colors[1]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Compass Z", new PointPairList(), colors[2]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        pane.Title.Text = "Magnetometer";
                        pane.YAxis.Title.Text = "mG";
                        break;
                    }
                case 9:
                    {
                        LineItem curve = pane.AddCurve("Motor FL", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Motor FR", new PointPairList(), colors[1]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Motor RL", new PointPairList(), colors[2]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Motor RR", new PointPairList(), colors[3]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        pane.Title.Text = "Motor PWMs";
                        pane.YAxis.Title.Text = "PWM [%]";
                        break;
                    }
                case 10:
                    {
                        LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;
                        
                        pane.Title.Text = "Position";
                        pane.YAxis.Title.Text = "cm";
                        break;
                    }
                case 11:
                    {
                        LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
                        curve.Symbol.Type = SymbolType.None;
                        curve.Line.IsOptimizedDraw = true;

                        pane.Title.Text = "Velocity";
                        pane.YAxis.Title.Text = "cm/s";
                        break;
                    }
                default:
                    break;
            }
        }

        private void cbbItem_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            UpdateGraphPane();
            drawGraphs();
        }
    }
}
