using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
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
using System.Windows.Navigation;
using System.Xml;
using System.Xml.Linq;
using System.Threading;
using System.IO;
using System.Diagnostics;
using System.Drawing;

using CustomExtensions;
using ZedGraph;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Graphics;
using TechJectDF;
using TechJectDF.TJCommands;

using Xceed.Wpf.Toolkit;
using System.Collections;


namespace AlphaUtilityWF
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // =================================================================== Global Variables ===================================================================
        byte chID = 1; // Channel ID, defult ID is 1
        Stopwatch stopwatch;

        // OpenTK display
        TJStatePacket globalState = null;
        GLControl glcAttitude;

        //long last3DViewUpdate = 0;
        const long k3DViewUpdatePeriod = 25;
        //bool Dragonfly3DViewLoaded = false;

        //static float[] light0_color = { 0f, 1f, 0f, 1f };
        //float[] light0Position = { 1.5f, -5f, 3.2f, 50f };

        //Matrix4 matrixProjection;
        //Matrix4 matrixModelview;
        Matrix4 roll_matrix = Matrix4.Identity;
        Matrix4 pitch_matrix = Matrix4.Identity;
        Matrix4 yaw_matrix = Matrix4.Identity;
        Matrix4 rotation_matrix = Matrix4.Identity;
        Matrix4 view_matrix = Matrix4.Identity;

        Vector4 lightPos_world = new Vector4(0f, 0f, 0f, 1f);

        double roll_angle, pitch_angle; // Angles in radium

        // Engineering graphs
        WindowsFormsHost[] winFormHosts;
        WindowsFormsHost[] winFormHosts2;
        ZedGraphControl[] graphs;
        ZedGraphControl[] graphs2;

        ZedGraphControl AccelGraph = new ZedGraphControl();
        ZedGraphControl GyroGraph = new ZedGraphControl();
        ZedGraphControl VoltageGraph = new ZedGraphControl();
        ZedGraphControl MotorGraph = new ZedGraphControl();
        ZedGraphControl PressureGraph = new ZedGraphControl();
        ZedGraphControl TemperatureGraph = new ZedGraphControl();
        ZedGraphControl HumidityGraph = new ZedGraphControl();

        ZedGraphControl CompassGraph = new ZedGraphControl();
        ZedGraphControl GPSGraph = new ZedGraphControl();

        ZedGraphControl MagGraph = new ZedGraphControl();
        ZedGraphControl MotorsGraph = new ZedGraphControl();
        ZedGraphControl PosGraph = new ZedGraphControl();
        ZedGraphControl VelGraph = new ZedGraphControl();

        public const float kAccelScaleFactor = 1 / 256.0f;
        public const float kGyroScaleFactor = 8.75f / 1000.0f; //100.0f/(2 << 13);
        public const float kAvdd = 3.0f;
        public const float kADCResolution = 4096f;
        public const float kADCScaleFactor = kAvdd / kADCResolution;
        public const float kTempScaleFactor = 1.0f / 10.0f;
        public const float VoltageScaleFactor = kADCScaleFactor * 2.0f;
        public const float kServoScaleFactor = kADCScaleFactor * 4f / 3f;
        public const float kSupplyScaleFactor = kADCScaleFactor * 4f / 3f;

        // Because the packet can only contain integer bytes. The float numbers are amplified.
        public const float kOFCScaleFactor = 1 / 100.0f;
        public const float kAttitudeScalerFactor = 1 / 100.0f;
        public const float kRefAngleScalerFactor = 1 / 100.0f;

        public const float kPWMScalerFactor = 1 / 60.0f; // for dc motor quad
        //public const float kPWMBLDCScalerFactor = 1 / 20.0f; // for bldc motor quad
        //public const float kServoMinScalerFactor = 1875; // for bldc motor quad

        public const float kPosScalerFactor = 1 / 100.0f;

        const float AccelValueMin = -2.0f;
        const float AccelValueMax = 2.0f;
        const float GyroValueMin = -200.0f;
        const float GyroValueMax = 200.0f;
        const float VoltageValueMin = -1.0f;
        const float VoltageValueMax = 5.0f;
        const float MotorsValueMin = 0.0f;
        const float MotorsValueMax = 100.0f;

        long lastGraphUpdate = 0;
        int maxPointsInGraph = 350;
        long graphUpdatePeriod = 50;  // milliseconds
        long packetRateScaler = 10; //30
        long packetIgnoreCount = 0;

        long lastGraph2Update = 0;
        long packet2IgnoreCount = 0;

        LinkedList<Tuple<TJStatePacket, long>> bufferedStates = new LinkedList<Tuple<TJStatePacket, long>>();
        LinkedList<Tuple<TJState2Packet, long>> bufferedStates2 = new LinkedList<Tuple<TJState2Packet, long>>();

        // PID tuning
        TJPIDControlGains[][] gains;
        System.Windows.Controls.TextBox[][][] gainTextboxes;
        System.Windows.Controls.Slider[][][] gainSliders;
        const int rollIndex = 0;
        const int pitchIndex = 1;
        const int yawIndex = 2;
        const int kpIndex = 0;
        const int kiIndex = 1;
        const int kdIndex = 2;

        Stopwatch tuningTimer = new Stopwatch();
        KnobControl yawKnob = new KnobControl();

        // Optical flow
        System.Windows.Forms.PictureBox pictureBoxCam = new System.Windows.Forms.PictureBox();
        LinkedList<TJImage> ImageList = new LinkedList<TJImage>();
        LinkedList<Bitmap> BitmapList = new LinkedList<Bitmap>();
        Bitmap KeepOriginal = null;
        const int MaximumFrameCount = 10;
        double AspectRatio = 4 / 3;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            stopwatch = Stopwatch.StartNew();

            // Create the GLControl.
            glcAttitude = new GLControl();
            glcAttitude.Load += new EventHandler(glcAttitude_Load); // initialization
            glcAttitude.Paint += new System.Windows.Forms.PaintEventHandler(glcAttitude_Paint); // update/display graph continuously
            wfhAttitude.Child = glcAttitude;

            // Setup engineering graphs
            winFormHosts = new[]
            {
                wfhAccelGraph,
                wfhGyroGraph,
                wfhVoltageGraph,
                wfhMotorGraph,
                wfhPressureGraph,
                wfhHumidtyGraph,
                wfhTemperatureGraph,
                wfhCompassGraph,
                wfhGpsGraph,
            };

            winFormHosts2 = new[]
            {
                //wfhCompassGraph,
                //wfhGpsGraph,
                wfhMagGraph,
                wfhMotorsGraph,
                wfhPosGraph,
                wfhVelGraph,
            };

            graphs = new[]
            {
                AccelGraph,
                GyroGraph,
                VoltageGraph,
                MotorGraph,
                PressureGraph,
                HumidityGraph,
                TemperatureGraph,
                CompassGraph,
                GPSGraph,
            };

            graphs2 = new[]
            {
                //CompassGraph,
                //GPSGraph
                MagGraph,
                MotorsGraph,
                PosGraph,
                VelGraph,
            };


            for (int i = 0; i < winFormHosts.Length; i++)
                winFormHosts[i].Child = graphs[i];

            for (int i = 0; i < winFormHosts2.Length; i++)
                winFormHosts2[i].Child = graphs2[i];

            // Setup Controls
            SetupcbbSavePacketFormat();
            SetupknbYawTrim();
            SetupGraphs(); // set up initial labels/texts
            SetupGraphs2(); // set up initial labels/texts for state packet 2
            SetupGainControls();

            wfhDFYawTrim.Child = yawKnob;
            wfhCamImage.Child = pictureBoxCam;

            // Connect to Dragonfly automatically 
            ConnectToDragonfly();
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            TJDragonfly.Disconnect();
        }

        void SetupcbbSavePacketFormat()
        {
            List<TJPacketExporter.TJPacketExportFormat> data = new List<TJPacketExporter.TJPacketExportFormat>();
            data.Add(TJPacketExporter.TJPacketExportFormat.CSV);
            data.Add(TJPacketExporter.TJPacketExportFormat.Binary);

            // ... Assign the ItemsSource to the List.
            cbbSavePacketFormat.ItemsSource = data;
            cbbSavePacketFormat.SelectedIndex = 0;
        }

        private void SetupknbYawTrim()
        {
            yawKnob.ForeColor = System.Drawing.Color.Black;
            yawKnob.BackColor = System.Drawing.Color.FromArgb(234, 235, 238);
            yawKnob.ImeMode = System.Windows.Forms.ImeMode.On;
            yawKnob.Location = new System.Drawing.Point(8, 176);
            yawKnob.Maximum = 90;
            yawKnob.Minimum = 0;
            yawKnob.SmallChange = 5;
            yawKnob.Name = "knbYawTrim";
            yawKnob.ShowLargeScale = false;
            yawKnob.ShowSmallScale = true;
            yawKnob.Size = new System.Drawing.Size(95, 95);
            yawKnob.TabIndex = 0;
            yawKnob.Value = 45;
            yawKnob.ValueChanged += new AlphaUtilityWF.ValueChangedEventHandler(this.YawTrim_ValueChanged);
        }

        /// <summary>
        /// Make the window draggable
        /// </summary>
        /// <param name="e"></param>
        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            base.OnMouseLeftButtonDown(e);

            // Begin dragging the window
            this.DragMove();
        }

        /// <summary>
        /// Callback function to the click event of the connect button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (!TJDragonfly.IsConnected)
            {
                ConnectToDragonfly();
            }
            else
            {
                //if (this.btnCamera.Content.ToString() == "Disable Camera")
                //{
                //    this.btnCamera.Content = "Enable Camera";
                //    TJCommand cmd = new TJCommand(TJCommandID.ToggleCameraCmdID);
                //    TJDragonfly.EnqueueCommand(cmd);

                //    this.btnGetFrame.IsEnabled = false;
                //}
                if (cbbPosHld.SelectedIndex == 1 || cbbCam.SelectedIndex == 1)
                {
                    cbbPosHld.SelectedIndex = 0;
                    cbbCam.SelectedIndex = 0;

                    byte[] robotPacket = new byte[32];
                    robotPacket[0] = (byte)TJCommandID.EnableDisableCameraCmdID;
                    robotPacket[2] = (byte)0;

                    btnGetFrame.IsEnabled = false;

                    TJDragonfly.EnqueueTXPacket(robotPacket);
                }

                if (cbbPx4flow.SelectedIndex == 1)
                {
                    cbbPx4flow.SelectedIndex = 0;

                    byte[] robotPacket = new byte[32];
                    robotPacket[0] = (byte)TJCommandID.EnableDisablePX4CmdID;
                    robotPacket[2] = (byte)0;

                    TJDragonfly.EnqueueTXPacket(robotPacket);
                }

                // Disconnect the robot
                TJDragonfly.Disconnect();
            }
        }

        private void btnUpdateFirmware_Click(object sender, RoutedEventArgs e)
        {
            TJDragonfly.Disconnect();
            TJDragonfly.DragonflyConnectionChanged -= TJDragonfly_DragonflyConnectionChanged;
            TJDragonfly.DragonflyStateUpdate -= UpdateUIState;
            new Bootloader(chID).Show();
        }

        // Commented out for now
        // Used for switch communication channel
        //private void btnSetAddress_Click(object sender, RoutedEventArgs e)
        //{
        //    btnSetAddress.IsEnabled = false;

        //    //if (!TJDragonfly.IsConnected)
        //    //    ConnectToDragonfly();

        //    //if (!TJDragonfly.IsConnected)
        //    //    return;

        //    try
        //    {
        //        // Set command to robot to change address
        //        //int neg = -1;
        //        //byte[] robotPacket = new byte[32];
        //        //robotPacket[0] = (byte)TJCommandID.SetAddressCmdID;
        //        //robotPacket[2] = (byte)(cbbRobotID.SelectedIndex);
        //        //robotPacket[3] = (byte)neg;
        //        //robotPacket[4] = (byte)neg;
        //        //robotPacket[5] = (byte)neg;
        //        //TJDragonfly.EnqueueTXPacket(robotPacket);

        //        // Change the nRF address on SDK
        //        switch (chID)
        //        {
        //            case 0:
        //                TJnRFController.SetRFRX((byte)TechJectDF.TJnRFController.nRFChID.chID0); break;
        //            case 1:
        //                TJnRFController.SetRFRX((byte)TechJectDF.TJnRFController.nRFChID.chID1); break;
        //            case 2:
        //                TJnRFController.SetRFRX((byte)TechJectDF.TJnRFController.nRFChID.chID2); break;
        //            case 3:
        //                TJnRFController.SetRFRX((byte)TechJectDF.TJnRFController.nRFChID.chID3); break;
        //            case 4:
        //                TJnRFController.SetRFRX((byte)TechJectDF.TJnRFController.nRFChID.chID4); break;
        //            case 5:
        //                TJnRFController.SetRFRX((byte)TechJectDF.TJnRFController.nRFChID.chID5); break;
        //            default:
        //                break;
        //        }

        //        Thread.Sleep(800);

        //        Console.WriteLine("nRF has been changed to ID " + cbbRobotID.SelectedIndex);
        //    }
        //    catch (Exception)
        //    {
        //        Console.WriteLine("nRF ID change failed, please try again.");
        //    }

        //    btnSetAddress.IsEnabled = true;
        //}

        //private void cbbRobotID_SelectionChanged(object sender, SelectionChangedEventArgs e)
        //{
        //    chID = (byte)(cbbRobotID.SelectedIndex);
        //}

        /// <summary>
        /// This method responds to any changes in the status of the connection to the dragonfly.
        /// </summary>
        void TJDragonfly_DragonflyConnectionChanged()
        {
            if (TJDragonfly.IsConnected)
            {
                ClearGraphs();
                ClearGraphs2();
                TJDragonfly.DragonflyStateUpdate += UpdateUIState;
                TJDragonfly.DragonflyState2Update += UpdateUIState2;
                TJDragonfly.DragonflyCameraUpdate += UpdateCameraFrame;
                TJDragonfly.DragonflyStartFrame += InitializeFrame;
                TJDragonfly.DragonflyEndFrame += DisplayFrame;
                TJCommand cmd = new TJCommand(TJCommandID.SetTogglesCmdID);
                TJDragonfly.EnqueueCommand(cmd);
                btnConnect.Content = "Disconnect";
            }
            else
            {
                TJDragonfly.DragonflyConnectionChanged -= TJDragonfly_DragonflyConnectionChanged;
                TJDragonfly.DragonflyStateUpdate -= UpdateUIState;
                TJDragonfly.DragonflyState2Update -= UpdateUIState2;
                TJDragonfly.DragonflyCameraUpdate -= UpdateCameraFrame;
                TJDragonfly.DragonflyStartFrame -= InitializeFrame;
                TJDragonfly.DragonflyEndFrame -= DisplayFrame;
                btnConnect.Content = "Connect";
            }
        }

        /// <summary>
        /// This method displays a captured image after the dragonfly has received an "EndFrame" packet.
        /// </summary>
        private void DisplayFrame()
        {
            // Reset "original"
            KeepOriginal = null;

            // Create Bitmap
            Bitmap ImageBitMap = new Bitmap(ImageList.Last.Value.Columns, ImageList.Last.Value.Rows, System.Drawing.Imaging.PixelFormat.Format8bppIndexed);

            // Create grayscale palette
            System.Drawing.Imaging.ColorPalette TJPalette = ImageBitMap.Palette;
            for (int i = 0; i < 256; i++)
            {
                TJPalette.Entries[i] = Color.FromArgb((int)255, i, i, i);
            }

            // Assign the palette to the bitmap
            ImageBitMap.Palette = TJPalette;

            // Lock it to get the Bitmap Data object
            System.Drawing.Imaging.BitmapData ImageData = ImageBitMap.LockBits(new Rectangle(0, 0, ImageBitMap.Width, ImageBitMap.Height), System.Drawing.Imaging.ImageLockMode.ReadWrite, ImageBitMap.PixelFormat);

            // Modified here by Yangbo to correctly display all the frame size, 12/18/2014
            if (ImageList.Last.Value.Columns % 4 == 0)
            {
                // Get address of first line
                IntPtr ptr = ImageData.Scan0;
                // Copy the array into the bitmap, arguments: byte[] source, int startIndex, IntPtr destination, int length
                System.Runtime.InteropServices.Marshal.Copy(ImageList.Last.Value.ImageArray, 0, ptr, ImageList.Last.Value.ImageArray.Length);
            }
            else
            // cols cannot be divided by 4
            {
                for (int i = 0; i < ImageList.Last.Value.Rows; i++)
                {
                    IntPtr ptr = ImageData.Scan0 + i * (ImageList.Last.Value.Columns + (4 - ImageList.Last.Value.Columns % 4));
                    System.Runtime.InteropServices.Marshal.Copy(ImageList.Last.Value.ImageArray, 0 + i * ImageList.Last.Value.Columns, ptr, ImageList.Last.Value.Columns);
                }
            }

            // Unlock the bitmap
            ImageBitMap.UnlockBits(ImageData);

            // Put this bitmap into the displayed bitmap memory
            BitmapList.AddLast(ImageBitMap);

            // Display
            this.Dispatcher.BeginInvoke((Action)delegate()
            {
                pictureBoxCam.Width = ImageBitMap.Width;
                pictureBoxCam.Image = ImageBitMap;
                btnGetFrame.IsEnabled = true;
            });
        }

        public static BitmapSource loadBitmap(System.Drawing.Bitmap source)
        {
            IntPtr ip = source.GetHbitmap();
            BitmapSource bs = null;
            try
            {
                bs = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(ip,
                   IntPtr.Zero, Int32Rect.Empty,
                   System.Windows.Media.Imaging.BitmapSizeOptions.FromEmptyOptions());
            }
            catch (Exception)
            {
                throw;
            }

            return bs;
        }

        /// <summary>
        /// This method attempts to connect the SDK to a dragonfly
        /// </summary>
        void ConnectToDragonfly()
        {
            TJDragonfly.DragonflyConnectionChanged += TJDragonfly_DragonflyConnectionChanged;
            TJDragonfly.nRFChID = chID;
            TJDragonfly.Connect();
        }

        /// <summary>
        /// This method initiates the display for gyroscope
        /// </summary>
        void glcAttitude_Load(object sender, EventArgs e)
        {
            GL.ClearColor(System.Drawing.Color.SkyBlue);

            int w = (int)wfhAttitude.Width;
            int h = (int)wfhAttitude.Height;

            // Set up initial modes
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            GL.Ortho(0, w, 0, h, -1, 1);
            GL.Viewport(0, 0, w, h);
        }

        /// <summary>
        /// This method implements the graphic display for attitude
        /// </summary>
        void glcAttitude_Paint(object sender, System.Windows.Forms.PaintEventArgs e)
        {
            this.Dispatcher.BeginInvoke((Action)delegate()
            {
                GLControl glc = sender as GLControl;

                // Adjust the projection view
                int w = (int)glc.Width;
                int h = (int)glc.Height;
                GL.MatrixMode(MatrixMode.Projection);
                GL.LoadIdentity();
                GL.Ortho(0, w, 0, h, -1, 1);
                GL.Viewport(0, 0, w, h);

                if (Math.Abs(roll_angle) >= 1.4)
                    return; // Constrain the display attitude

                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                GL.MatrixMode(MatrixMode.Modelview);
                GL.LoadIdentity();

                if (globalState != null)
                {
                    double scale_k = 100;
                    double center_h = h / 2;
                    double center_w = w / 2;
                    double a = center_h + pitch_angle * scale_k;
                    double b = a - Math.Tan(roll_angle) * center_w; // Left edge
                    double c = a + Math.Tan(roll_angle) * center_w; // Right edge

                    // Draw the ground
                    GL.Color3(System.Drawing.Color.Green);
                    GL.Begin(BeginMode.Polygon);
                    if (b < 0)
                        GL.Vertex2(w, 0);
                    else if (c < 0)
                        GL.Vertex2(0, 0);
                    else
                    {
                        GL.Vertex2(0, 0);
                        GL.Vertex2(w, 0);
                    }
                    GL.Vertex2(w, c);
                    GL.Vertex2(0, b);
                    GL.End();

                    // Draw the scales
                    GL.Color3(System.Drawing.Color.Black);
                    GL.Begin(BeginMode.Lines);
                    GL.Vertex2(center_w, 0);
                    GL.Vertex2(center_w, h);
                    GL.End();

                    int[] scales = { -100, -50, 0, 50, 100 };
                    for (int i = 0; i < scales.Length; i++)
                    {
                        double temp = scales[i] * Math.PI / 180 * scale_k;

                        GL.Color3(System.Drawing.Color.Black);
                        GL.Begin(BeginMode.Lines);
                        GL.Vertex2(center_w - 20, center_h + temp);
                        GL.Vertex2(center_w + 20, center_h + temp);
                        GL.End();

                        //drawString(Math.Abs(scales[i]).ToString(), center_w, center_h + temp, w, h);
                    }

                    // Draw the center cross
                    GL.Color3(System.Drawing.Color.White);
                    GL.Begin(BeginMode.Lines);
                    GL.Vertex2(0, a);
                    GL.Vertex2(w, a);
                    GL.End();

                    sldAttitudePitch.Minimum = -center_h;
                    sldAttitudePitch.Maximum = center_h;
                    sldAttitudePitch.Value = a - center_h;

                    tbxRollAngle.Text = (roll_angle / Math.PI * 180).ToString();
                    tbxPitchAngle.Text = (pitch_angle / Math.PI * 180).ToString();

                    GL.Begin(BeginMode.Lines);
                    GL.Vertex2(center_w, a - 20);
                    GL.Vertex2(center_w, a + 20);
                    GL.End();
                }

                glc.SwapBuffers();

            });
        }

        //void drawString(string str, double x, double y, double w, double h)
        //{
        //    using (QFont font = new QFont("Fonts/times.ttf", 20, new QFontBuilderConfiguration(true)))
        //    {
        //        font.Options.DropShadowActive = false;
        //        QFont.Begin();
        //        GL.PushMatrix();
        //        GL.Translate(x + 30, h - y - 10, 0f);
        //        font.Print(str, QFontAlignment.Left);
        //        GL.PopMatrix();
        //        QFont.End();
        //        GL.Disable(EnableCap.Texture2D);
        //    }
        //}

        /// <summary>
        /// This method triggers all the display functions to update graphs.
        /// </summary>
        private void UpdateDisplay(TJStatePacket state)
        {
            double Z = 255 - Math.Sqrt(Math.Abs(state.Accel.X * state.Accel.X + state.Accel.Y * state.Accel.Y));
            roll_angle = Math.Atan2(state.Accel.X, Z);
            pitch_angle = Math.Atan2(state.Accel.Y, Z);

            glcAttitude.Invalidate();

            //pieTemp.EndAngle = pieTemp.StartAngle + 30 + state.Temperature.Temperature * 0.3;
            //pieHumidity.EndAngle = pieHumidity.StartAngle + 30 + state.Humidity.Humidity;

        }

        // =================================================================== Engineering View 1 Tab ===================================================================
        /// <summary>
        /// Sets up the graphs
        /// </summary>
        void SetupGraphs()
        {
            SetupAccelGraph();
            SetupGyroGraph();
            SetupVoltageGraph();
            SetupMotorGraph();
            SetupPressureGraph();
            SetupTemperatureGraph();
            SetupHumidityGraph();

            SetupCompassGraph();
            SetupGPSGraph();
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the accelerometer graph
        /// </summary>
        void SetupAccelGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet };
            GraphPane pane = AccelGraph.GraphPane;

            LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Z", new PointPairList(), colors[2]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Accelerometer";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "g";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the gyroscope graph
        /// </summary>
        void SetupGyroGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet };
            GraphPane pane = GyroGraph.GraphPane;

            LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Z", new PointPairList(), colors[2]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Gyroscope";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "dps";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the voltage graph
        /// </summary>
        void SetupVoltageGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet };
            GraphPane pane = VoltageGraph.GraphPane;

            LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            //curve = pane.AddCurve("Extra", new PointPairList(), colors[2]);
            //curve.Symbol.Type = SymbolType.None;
            //curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "OF Outputs";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "dps";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the motor PWM output graph
        /// </summary>
        void SetupMotorGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.DarkOliveGreen };
            GraphPane pane = MotorGraph.GraphPane;

            LineItem curve = pane.AddCurve("Ref Pitch", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Ref Roll", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            //curve = pane.AddCurve("RL", new PointPairList(), colors[2]);
            //curve.Symbol.Type = SymbolType.None;
            //curve.Line.IsOptimizedDraw = true;

            //curve = pane.AddCurve("RR", new PointPairList(), colors[3]);
            //curve.Symbol.Type = SymbolType.None;
            //curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Reference Angles";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "deg";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the pressure output graph
        /// </summary>
        void SetupPressureGraph()
        {
            Color[] colors = { Color.DarkBlue };
            GraphPane pane = PressureGraph.GraphPane;

            LineItem curve = pane.AddCurve("Roll", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve.Line.Width = 2;

            pane.Title.Text = "Roll";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "deg";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.Min = -30;
            pane.YAxis.Scale.Max = 30;
            pane.YAxis.Scale.MajorStep = 5;
            pane.YAxis.Scale.MinorStep = 2.5;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the humidity output graph
        /// </summary>
        void SetupHumidityGraph()
        {
            Color[] colors = { Color.DarkBlue };
            GraphPane pane = HumidityGraph.GraphPane;

            LineItem curve = pane.AddCurve("Pitch", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve.Line.Width = 2;

            pane.Title.Text = "Pitch";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "deg";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.Min = -30;
            pane.YAxis.Scale.Max = 30;
            pane.YAxis.Scale.MajorStep = 5;
            pane.YAxis.Scale.MinorStep = 2.5;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the light output graph
        /// </summary>
        void SetupTemperatureGraph()
        {
            Color[] colors = { Color.DarkBlue };
            GraphPane pane = TemperatureGraph.GraphPane;

            LineItem curve = pane.AddCurve("Yaw", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve.Line.Width = 2;

            pane.Title.Text = "Yaw";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "deg";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.Min = -30;
            pane.YAxis.Scale.Max = 30;
            pane.YAxis.Scale.MajorStep = 5;
            pane.YAxis.Scale.MinorStep = 2.5;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the compass output graph
        /// </summary>
        void SetupCompassGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.DarkOliveGreen };
            GraphPane pane = CompassGraph.GraphPane;

            LineItem curve = pane.AddCurve("Gyro X", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("OF Output X", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            //curve = pane.AddCurve("Z", new PointPairList(), colors[2]);
            //curve.Symbol.Type = SymbolType.None;
            //curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Gyro x V.S. OF x  --  Pitch";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "dps";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the GPS output graph
        /// </summary>
        void SetupGPSGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.DarkOliveGreen };
            GraphPane pane = GPSGraph.GraphPane;

            LineItem curve = pane.AddCurve("Gyro Y", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("OF Output Y", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Gyro y V.S. OF y  --  Roll";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "dps";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        // =================================================================== Engineering View 2 Tab ===================================================================
        /// <summary>
        /// Sets up the graphs for state pachet 2
        /// </summary>
        void SetupGraphs2()
        {
            SetupMagGraph();
            SetupMotorsGraph();
            SetupPosGraph();
            SetupVelGraph();
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the Magnetometer output graph (in state packet 2)
        /// </summary>
        void SetupMagGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.DarkOliveGreen };
            GraphPane pane = MagGraph.GraphPane;

            LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Z", new PointPairList(), colors[2]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Magnetometer";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "mG";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the motor PWM output graph
        /// </summary>
        void SetupMotorsGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.DarkOliveGreen };
            GraphPane pane = MotorsGraph.GraphPane;

            LineItem curve = pane.AddCurve("FL", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("FR", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("RL", new PointPairList(), colors[2]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("RR", new PointPairList(), colors[3]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Motor PWMs";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "PWM [%]";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the position graph
        /// </summary>
        void SetupPosGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.DarkOliveGreen };
            GraphPane pane = PosGraph.GraphPane;

            LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;
            
            pane.Title.Text = "Position";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "cm";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// Sets up the zedgraph control corresponding to the velocity graph
        /// </summary>
        void SetupVelGraph()
        {
            Color[] colors = { Color.DarkBlue, Color.DarkOrange, Color.DarkViolet, Color.DarkOliveGreen };
            GraphPane pane = VelGraph.GraphPane;

            LineItem curve = pane.AddCurve("X", new PointPairList(), colors[0]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            curve = pane.AddCurve("Y", new PointPairList(), colors[1]);
            curve.Symbol.Type = SymbolType.None;
            curve.Line.IsOptimizedDraw = true;

            pane.Title.Text = "Velocity";
            pane.XAxis.Title.Text = "sec";
            pane.YAxis.Title.Text = "cm/s";
            pane.XAxis.Scale.FontSpec.Size = 20;
            pane.XAxis.Title.FontSpec.Size = 20;
            pane.YAxis.Scale.FontSpec.Size = 20;
            pane.YAxis.Title.FontSpec.Size = 20;
            pane.Legend.FontSpec.Size = 20;
            pane.Title.FontSpec.Size = 20;
            pane.Fill = new Fill(Color.FromArgb(213, 216, 221));
            pane.Border.Color = Color.FromArgb(213, 216, 221);
            pane.Title.IsVisible = false;
        }

        /// <summary>
        /// This method updates the displayed graphs after receiving the appropriate number of packets.
        /// Steps: 1. encapsulate time and state values into PointPair object;
        ///        2. append the encapsulated pointpair into the curves of a ZedGraph object
        ///        3. update graph in the front end
        /// </summary>
        void UpdateGraphs(LinkedList<Tuple<TJStatePacket, long>> states)
        {
            foreach (Tuple<TJStatePacket, long> pair in states)
            {
                TJStatePacket state = pair.Item1; // item1: payload
                double xValue = pair.Item2 / 1000.0f; // item2: time

                PointPair[][] points = // combine time and state value into pairs
                {
                    new[]
                    { 
                        new PointPair(xValue, state.Accel.X * kAccelScaleFactor),
                        new PointPair(xValue, state.Accel.Y * kAccelScaleFactor),
                        new PointPair(xValue, state.Accel.Z * kAccelScaleFactor)
                    },
                    new[]
                    {
                        new PointPair(xValue, state.Gyro.X * kGyroScaleFactor),
                        new PointPair(xValue, state.Gyro.Y * kGyroScaleFactor),
                        new PointPair(xValue, state.Gyro.Z * kGyroScaleFactor),
                    },
                    new[]
                    {
                        // voltage data are overwritten by OFC outputs
                        new PointPair(xValue, state.Batt.LiBatt * kOFCScaleFactor), // vy
                        new PointPair(xValue, state.Batt.Servo * kOFCScaleFactor),  // vx
                        //new PointPair(xValue, state.Batt.Supply * kOFCScaleFactor)
                    },
                    new[] 
                    {
                        new PointPair(xValue, state.Motors.FrontLeft * kRefAngleScalerFactor), // reference pitch
                        new PointPair(xValue, state.Motors.FrontRight * kRefAngleScalerFactor),// reference roll
                        //new PointPair(xValue, state.Motors.RearLeft * kRefAngleScalerFactor),
                        //new PointPair(xValue, state.Motors.RearRight * kRefAngleScalerFactor)
                    },
                    new[] 
                    {
                        // Attitude roll
                        new PointPair(xValue, state.Pressure.Pressure * kAttitudeScalerFactor)
                    },
                    new[]
                    {
                        // Attitude pitch
                        new PointPair(xValue, state.Humidity.Humidity * kAttitudeScalerFactor)
                    },
                    new[]
                    {
                        // Attitude yaw
                        new PointPair(xValue, state.Temperature.Temperature * kAttitudeScalerFactor)
                    },
                    new[] 
                    { 
                        // Gyro Output X vs. OFC Output X
                        new PointPair(xValue, state.Gyro.X * kGyroScaleFactor),
                        new PointPair(xValue, state.Batt.LiBatt * kOFCScaleFactor)
                    },
                    new[] 
                    { 
                        // Gyro Output Y vs. OFC Output Y
                        new PointPair(xValue, state.Gyro.Y * kGyroScaleFactor),
                        new PointPair(xValue, state.Batt.Servo * kOFCScaleFactor * (-1))
                    },
                };

                for (int i = 0; i < graphs.Length; i++) // add pointpair into ZedGraph object to plot data
                {
                    CurveList curveList = graphs[i].GraphPane.CurveList;
                    CurveItem curve = graphs[i].GraphPane.CurveList[0]; // this line is useless
                    for (int j = 0; j < curveList.Count; j++)
                    {
                        IPointListEdit ip = (IPointListEdit)graphs[i].GraphPane.CurveList[j].Points;
                        ip.Add(points[i][j]); // append the encapsulated pointpair into the curves of a ZedGraph object

                        if (ip.Count > maxPointsInGraph)
                            ip.RemoveAt(0); // cannot display more points, so remove the left-most point
                    }
                }
            }

            for (int i = 0; i < graphs.Length; i++) // refresh/update the appearance of the graph in the frontend
            {
                graphs[i].AxisChange();
                graphs[i].Refresh();
            }
        }


        /// <summary>
        /// This method updates the displayed graphs after receiving the appropriate number of packets.
        /// </summary>
        void UpdateGraphs2(LinkedList<Tuple<TJState2Packet, long>> states)
        {
            foreach (Tuple<TJState2Packet, long> pair in states)
            {
                TJState2Packet state = pair.Item1;
                double xValue = pair.Item2 / 1000.0f;

                PointPair[][] points = 
                {
                    new[]
                    {
                        new PointPair(xValue, state.Compass.X - 10000), // -10000 since the raw data were added by 10000 before transmitting from MARC
                        new PointPair(xValue, state.Compass.Y - 10000),
                        new PointPair(xValue, state.Compass.Z - 10000)
                    },
                    new[]
                    {
                        // for dc motor quad
                        new PointPair(xValue, state.PWMs.FrontLeft * kPWMScalerFactor), // pwm values: 0 - 100
                        new PointPair(xValue, state.PWMs.FrontRight * kPWMScalerFactor),
                        new PointPair(xValue, state.PWMs.RearLeft * kPWMScalerFactor),
                        new PointPair(xValue, state.PWMs.RearRight * kPWMScalerFactor)
                        //// for bldc motor quad
                        //new PointPair(xValue, (state.PWMs.FrontLeft - kServoMinScalerFactor) * kPWMBLDCScalerFactor), // pwm values: 0 - 100
                        //new PointPair(xValue, (state.PWMs.FrontRight - kServoMinScalerFactor) * kPWMBLDCScalerFactor),
                        //new PointPair(xValue, (state.PWMs.RearLeft - kServoMinScalerFactor) * kPWMBLDCScalerFactor),
                        //new PointPair(xValue, (state.PWMs.RearRight - kServoMinScalerFactor) * kPWMBLDCScalerFactor)
                    },
                    new[]
                    {
                        new PointPair(xValue, state.Pos.X * kPosScalerFactor), 
                        new PointPair(xValue, state.Pos.Y * kPosScalerFactor)
                    },
                    new[] 
                    { 
                        new PointPair(xValue, state.Vel.X * kPosScalerFactor), 
                        new PointPair(xValue, state.Vel.Y * kPosScalerFactor)
                    }
                };

                for (int i = 0; i < graphs2.Length; i++)
                {
                    CurveList curveList = graphs2[i].GraphPane.CurveList;
                    //CurveItem curve = graphs2[i].GraphPane.CurveList[i]; // changed index from Curvelist[i] to Curvelist[0], don't know why this works
                    for (int j = 0; j < curveList.Count; j++)
                    {
                        IPointListEdit ip = (IPointListEdit)graphs2[i].GraphPane.CurveList[j].Points;
                        ip.Add(points[i][j]);

                        if (ip.Count > maxPointsInGraph)
                            ip.RemoveAt(0);
                    }
                }
            }

            for (int i = 0; i < graphs2.Length; i++)
            {
                graphs2[i].AxisChange();
                graphs2[i].Refresh();
            }
        }


        /// <summary>
        /// This method keeps track of received packets in order to notify UpdateGraphs of when it should execute.
        /// </summary>
        void UpdateUIState(TJStatePacket state)
        {
            long now = stopwatch.ElapsedMilliseconds;

            this.Dispatcher.BeginInvoke((Action)delegate()
            {
                // Handle timers
                // In PID tuning tab, if the time exceeds the reference time, stop it
                TimeSpan ts = tuningTimer.Elapsed; // assign the elapsed time to TimeSpan class, to display the time properly
                string elapsedTime = String.Format("{0:00}:{1:00}:{2:00}", ts.Hours, ts.Minutes, ts.Seconds);
                labPidTimer.Content = elapsedTime;
                labDFTimer.Content = elapsedTime;

                if (tbxPidTimerRef.Text != "" || tbxDFTimerRef.Text != "")
                {
                    try
                    {
                        int refTime = 0;

                        if (tbxPidTimerRef.Text != "")
                            refTime = Convert.ToInt16(tbxPidTimerRef.Text);
                        else
                            refTime = Convert.ToInt16(tbxDFTimerRef.Text);

                        if (ts.TotalSeconds >= refTime && tuningTimer.IsRunning)
                        {
                            // Stop throttle
                            byte[] throttlePacket = new byte[32];
                            throttlePacket[0] = 0x01;
                            throttlePacket[2] = (byte)0;
                            TJDragonfly.EnqueueTXPacket(throttlePacket);

                            sldPidThrottle.Value = 0;
                            labPidThrottleValue.Content = "0";

                            // Stop main motor
                            byte[] DFPacket = new byte[32];
                            DFPacket[0] = (byte)TJCommandID.DFNomSatCmdID;
                            DFPacket[2] = (byte)0;
                            DFPacket[3] = (byte)sldDFServoNom.Value;
                            DFPacket[4] = (byte)sldDFServoSat.Value;
                            DFPacket[5] = (byte)sldDFPitchSat.Value;
                            DFPacket[6] = (byte)sldDFYawSat.Value;
                            TJDragonfly.EnqueueTXPacket(DFPacket);

                            sldDFMainMotor.Value = 0;
                            labDFMainMotorValue.Content = "0";

                            // Stop timer
                            tuningTimer.Stop();

                            btnStart.IsEnabled = true;
                        }
                    }
                    catch (Exception)
                    {
                        //throw;
                    }
                }

                // Update Graphs
                if ((bool)cbxEnableGraphs.IsChecked)
                {
                    packetIgnoreCount += 1;

                    // See whether Aardvark is used
                    if (TJDragonfly.getRFController() == 0)
                    {
                        bufferedStates.AddLast(new Tuple<TJStatePacket, long>(state, now));
                        packetIgnoreCount = 0;
                    }
                    else if (TJDragonfly.getRFController() == 1)
                    {
                        if (packetIgnoreCount >= packetRateScaler)
                        {
                            bufferedStates.AddLast(new Tuple<TJStatePacket, long>(state, now));
                            packetIgnoreCount = 0;
                        }
                    }

                    if (now - lastGraphUpdate > graphUpdatePeriod)
                    {
                        lastGraphUpdate = now;

                        LinkedList<Tuple<TJStatePacket, long>> statesToAdd = bufferedStates;

                        this.Dispatcher.BeginInvoke((Action)delegate()
                        {
                            // Update global state 2
                            // Update graphic view
                            globalState = state;
                            UpdateDisplay(state);

                            // Update engineering view
                            UpdateGraphs(statesToAdd);
                        });

                        bufferedStates = new LinkedList<Tuple<TJStatePacket, long>>();
                    }
                }

                // Update altitude information
                //tbAltPanic.Text = (state.Altitide.Panic == 0) ? "No" : "Yes";
                tbAltPanic.Text = state.Altitide.Panic.ToString(); // to display target alt value

                if (cbbAltHold.SelectedIndex != 0) // display data in all three modes, auto take-off, landing, altitude hold
                    tbAltReading.Text = state.Altitide.Altitude.ToString();
                else // none of modes is selected
                {
                    tbAltReading.Text = "0";
                    //tbAltPanic.Text = "No";
                    tbAltPanic.Text = "0";
                }
            });


        }

        /// <summary>
        /// This method keeps track of received packets in order to notify UpdateGraphs of when it should execute.
        /// </summary>
        void UpdateUIState2(TJState2Packet state)
        {
            long now = stopwatch.ElapsedMilliseconds;

            this.Dispatcher.BeginInvoke((Action)delegate()
            {
                if ((bool)cbxEnableGraphs2.IsChecked)
                {
                    packet2IgnoreCount += 1;

                    // See whether Aardvark is used
                    if (TJDragonfly.getRFController() == 0)
                    {
                        bufferedStates2.AddLast(new Tuple<TJState2Packet, long>(state, now));
                        packet2IgnoreCount = 0;
                    }
                    else if (TJDragonfly.getRFController() == 1)
                    {
                        if (packet2IgnoreCount >= packetRateScaler)
                        {
                            bufferedStates2.AddLast(new Tuple<TJState2Packet, long>(state, now));
                            packet2IgnoreCount = 0;
                        }
                    }

                    if (now - lastGraph2Update > graphUpdatePeriod)
                    {
                        lastGraph2Update = now;

                        LinkedList<Tuple<TJState2Packet, long>> statesToAdd = bufferedStates2;

                        this.Dispatcher.BeginInvoke((Action)delegate()
                        {
                            UpdateGraphs2(statesToAdd);
                        });

                        bufferedStates2 = new LinkedList<Tuple<TJState2Packet, long>>();
                    }
                }
            });
        }

        /// <summary>
        /// This method inserts segments of an image frame into their appropriate locations as the image segments arrive via Camera Packets.
        /// </summary>
        void UpdateCameraFrame(TJCameraPacket packet)
        {
            if (ImageList.Count > 0)
            {
                ImageList.Last.Value.InsertSegment(packet);
            }
        }

        /// <summary>
        /// This method responds to the reception of a "StartFrame" packet by initialization an image array of the appropriate size.
        /// </summary>
        void InitializeFrame(TJStartFramePacket frame)
        {
            // Turn off button to avoid over-clicking
            this.Dispatcher.BeginInvoke((Action)delegate()
            {
                this.btnGetFrame.IsEnabled = false;
            });

            TJImage newFrame = new TJImage(frame);
            ImageList.AddLast(newFrame);

            while (ImageList.Count > MaximumFrameCount)
            {
                ImageList.RemoveFirst();
            }
        }

        /// <summary>
        /// This responds to the click event of the clear button
        /// </summary>
        private void btnClear_Click(object sender, RoutedEventArgs e)
        {
            ClearGraphs();
        }

        /// <summary>
        /// This method resets all graphs
        /// </summary>
        void ClearGraphs()
        {
            foreach (ZedGraphControl graph in graphs)
            {
                if (graph.GraphPane.CurveList.Count > 0)
                {
                    foreach (CurveItem cl in graph.GraphPane.CurveList)
                    {
                        IPointListEdit ip = (IPointListEdit)cl.Points;
                        ip.Clear();
                    }
                }
            }

            //foreach (ZedGraphControl graph in graphs2)
            //{
            //    if (graph.GraphPane.CurveList.Count > 0)
            //    {
            //        foreach (CurveItem cl in graph.GraphPane.CurveList)
            //        {
            //            IPointListEdit ip = (IPointListEdit)cl.Points;
            //            ip.Clear();
            //        }
            //    }
            //}
        }

        System.Windows.Forms.SaveFileDialog savePacketCapturedialog = new System.Windows.Forms.SaveFileDialog();

        /// <summary>
        /// This method respond to the click event of capture button
        /// </summary>
        private void btnSavePacketCapture_Click(object sender, RoutedEventArgs e)
        {
            TJPacketExporter.TJPacketExportFormat fmt = (TJPacketExporter.TJPacketExportFormat)cbbSavePacketFormat.SelectedValue;

            try
            {
                int packetsToCapture = Convert.ToInt32(tbxSavePacketNumber.Text);

                string filename = savePacketCapturedialog.FileName;
                if (packetsToCapture <= 0)
                {
                    System.Windows.Forms.MessageBox.Show("Choose a non-negative & non-zero value");
                    return;
                }

                if (filename == "")
                {
                    System.Windows.Forms.MessageBox.Show("Choose where to save the captured file");
                    return;
                }

                btnSavePacketCapture.IsEnabled = false;
                pbrCapture.Maximum = packetsToCapture;

                TJPacketExporter pexp = new TJPacketExporter();
                pexp.CaptureProgressChanged += PacketCapturer_CaptureProgressChanged;
                pexp.StartCapturing(filename, fmt, packetsToCapture);
            }
            catch (Exception)
            {
                //throw;
            }
        }

        /// <summary>
        /// This method will be called when the capturing packets is in progress
        /// </summary>
        void PacketCapturer_CaptureProgressChanged(object sender, TJPacketExporter.CaptureProgressEventArgs e)
        {
            this.InvokeIfRequired(() =>
            {
                pbrCapture.Value = e.NumberOfPacketsProcessed;

                if (e.NumberOfPacketsProcessed == e.MaxNumberOfPacketsToProcess)
                {
                    pbrCapture.Value = 0;
                    btnSavePacketCapture.IsEnabled = true;
                    Console.WriteLine("Done exporting packets");
                }
            });
        }

        /// <summary>
        /// Select the capture file, will pop up a window for location selection
        /// </summary>
        private void btnSelectCaptureFile_Click(object sender, RoutedEventArgs e)
        {
            savePacketCapturedialog.Title = "Select file location";
            savePacketCapturedialog.ShowDialog();
            if (savePacketCapturedialog.FileName != "")
            {
                tbxSavePacketFile.Text = savePacketCapturedialog.FileName;
            }
        }

        /// <summary>
        /// This method responds to the click event of plot graph button. It is used to plot offline graph of the data
        /// </summary>
        private void btnPlotOfflineGraphs_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.Forms.OpenFileDialog dialog = new System.Windows.Forms.OpenFileDialog();
            //raphDialog.Filter = "DAT-File | *.dat";
            dialog.Title = "Load data";
            dialog.ShowDialog();

            if (dialog.FileName == "")
                return;

            String input = File.ReadAllText(dialog.FileName);
            int i = 0, j = 0;
            List<List<int>> data = new List<List<int>>();
            foreach (var row in input.Split('\n'))
            {
                List<int> temp = new List<int>();
                foreach (var col in row.Trim().Split(','))
                {
                    if (col == "")
                        continue;

                    temp.Add(int.Parse(col.Trim()));
                }

                data.Add(temp);
            }

            new OfflineGraphsWindow(data).Show();
        }

        /// <summary>
        /// Increase the number of packets to be saved during capturing
        /// </summary>
        private void btnSavePacketUp_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                int value = Convert.ToInt32(tbxSavePacketNumber.Text);
                value += 10000;
                value = Math.Min(Int32.MaxValue, value);
                tbxSavePacketNumber.Text = value.ToString();
            }
            catch (Exception)
            {
                //throw;
            }
        }

        /// <summary>
        /// Decrease the number of packets to be saved during capturing
        /// </summary>
        private void btnSavePacketDown_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                int value = Convert.ToInt32(tbxSavePacketNumber.Text);
                value -= 10000;
                value = Math.Max(0, value);
                tbxSavePacketNumber.Text = value.ToString();
            }
            catch (Exception)
            {
                //throw;
            }
        }

        /// <summary>
        /// Utility function - Constrain the number within a given range
        /// </summary>
        private double constrain(double value, double min, double max)
        {
            if (value <= min)
                return min;
            else if (value >= max)
                return max;
            else
                return value;
        }

        // =================================================================== PID Tuning Tab ===================================================================
        /// <summary>
        /// Setup all the gain/PID controls
        /// </summary>
        private void SetupGainControls()
        {
            gains = new[]
            {   
                new[] {new TJPIDControlGains(), new TJPIDControlGains(), new TJPIDControlGains()}, // Roll
                new[] {new TJPIDControlGains(), new TJPIDControlGains(), new TJPIDControlGains()}, // Pitch
                new[] {new TJPIDControlGains(), new TJPIDControlGains(), new TJPIDControlGains()}, // Yaw
            };

            gainTextboxes = new[]
            {
                new[] 
                {
                    new[] {tbxPosRollKpValue, tbxPosRollKiValue,  tbxPosRollKdValue},
                    new[] {tbxPosPitchKpValue, tbxPosPitchKiValue,  tbxPosPitchKdValue},
                    new[] {tbxPosYawKpValue, tbxPosYawKiValue,  tbxPosYawKdValue},
                },
                new[] 
                {
                    new[] {tbxAglRollKpValue, tbxAglRollKiValue,  tbxAglRollKdValue},
                    new[] {tbxAglPitchKpValue, tbxAglPitchKiValue,  tbxAglPitchKdValue},
                    new[] {tbxAglYawKpValue, tbxAglYawKiValue,  tbxAglYawKdValue},
                },
                new[] 
                {
                    new[] {tbxAgrRollKpValue, tbxAgrRollKiValue,  tbxAgrRollKdValue},
                    new[] {tbxAgrPitchKpValue, tbxAgrPitchKiValue,  tbxAgrPitchKdValue},
                    new[] {tbxAgrYawKpValue, tbxAgrYawKiValue,  tbxAgrYawKdValue},
                },
            };

            gainSliders = new[]
            {
                new[] 
                {
                    new[] {sldPosRollKp, sldPosRollKi,  sldPosRollKd},
                    new[] {sldPosPitchKp, sldPosPitchKi,  sldPosPitchKd},
                    new[] {sldPosYawKp, sldPosYawKi,  sldPosYawKd},
                },
                new[] 
                {
                    new[] {sldAglRollKp, sldAglRollKi,  sldAglRollKd},
                    new[] {sldAglPitchKp, sldAglPitchKi,  sldAglPitchKd},
                    new[] {sldAglYawKp, sldAglYawKi,  sldAglYawKd},
                },
                new[] 
                {
                    new[] {sldAgrRollKp, sldAgrRollKi, sldAgrRollKd},
                    new[] {sldAgrPitchKp, sldAgrPitchKi, sldAgrPitchKd},
                    new[] {sldAgrYawKp, sldAgrYawKi, sldAgrYawKd},
                },
            };
        }

        /// <summary>
        /// Trigger the timer so that the throuttle and main motor controls are enabled
        /// </summary>
        private void btnStart_Click(object sender, RoutedEventArgs e)
        {
            if (!TJDragonfly.IsConnected)
                return;

            byte[] throttlePacket = new byte[32];
            throttlePacket[0] = (byte)TJCommandID.ThrottleCmdID;
            throttlePacket[2] = (byte)sldPidThrottle.Value;
            TJDragonfly.EnqueueTXPacket(throttlePacket);

            // arming motors when click on Start in SDK
            byte[] armingPacket = new byte[32];
            armingPacket[0] = (byte)TJCommandID.ArmingCmdID;
            armingPacket[2] = (byte)1;
            TJDragonfly.EnqueueTXPacket(armingPacket);

            labPidThrottleValue.Content = ((int)sldPidThrottle.Value).ToString();

            byte[] DFPacket = new byte[32];
            DFPacket[0] = (byte)TJCommandID.DFNomSatCmdID;
            DFPacket[2] = (byte)sldDFMainMotor.Value;
            DFPacket[3] = (byte)sldDFServoNom.Value;
            DFPacket[4] = (byte)sldDFServoSat.Value;
            DFPacket[5] = (byte)sldDFPitchSat.Value;
            DFPacket[6] = (byte)sldDFYawSat.Value;
            TJDragonfly.EnqueueTXPacket(DFPacket);

            labDFMainMotorValue.Content = ((int)sldDFMainMotor.Value).ToString();

            if (tbxPidTimerRef.Text != "")
                tbxDFTimerRef.Text = tbxPidTimerRef.Text;
            else if (tbxDFTimerRef.Text != "")
                tbxPidTimerRef.Text = tbxDFTimerRef.Text;

            btnStart.IsEnabled = false; // after clicking on the Start button, the button is turned gray/disabled
            btnStop.IsEnabled = true;

            tuningTimer.Reset();
            tuningTimer.Start();
        }

        /// <summary>
        /// E-stop, stop the throttle and main motor power
        /// </summary>
        private void btnStop_Click(object sender, RoutedEventArgs e)
        {
            byte[] throttlePacket = new byte[32];
            throttlePacket[0] = 0x01;
            throttlePacket[2] = (byte)0;
            TJDragonfly.EnqueueTXPacket(throttlePacket);

            // disarming motors when click on Stop in SDK
            byte[] armingPacket = new byte[32];
            armingPacket[0] = (byte)TJCommandID.ArmingCmdID;
            armingPacket[2] = (byte)0;
            TJDragonfly.EnqueueTXPacket(armingPacket);

            sldPidThrottle.Value = 0;
            labPidThrottleValue.Content = "0";

            byte[] DFPacket = new byte[32];
            DFPacket[0] = (byte)TJCommandID.DFNomSatCmdID;
            DFPacket[2] = (byte)0;
            DFPacket[3] = (byte)sldDFServoNom.Value;
            DFPacket[4] = (byte)sldDFServoSat.Value;
            DFPacket[5] = (byte)sldDFPitchSat.Value;
            DFPacket[6] = (byte)sldDFYawSat.Value;
            TJDragonfly.EnqueueTXPacket(DFPacket);

            sldDFMainMotor.Value = 0;
            labDFMainMotorValue.Content = "0";

            tuningTimer.Stop();

            btnStart.IsEnabled = true;
            //btnStop.IsEnabled = false;

            cbbAltHold.SelectedIndex = 0;
            tbAltReading.Text = "0";
            //tbAltPanic.Text = "No";
            tbAltPanic.Text = "0"; // to display target altitude

            // Send disable commands when E-stop is hit
            byte[] robotPacket = new byte[32];
            robotPacket[0] = (byte)TJCommandID.AltHoldCmdID;
            robotPacket[2] = (byte)0;
            TJDragonfly.EnqueueTXPacket(robotPacket);
            byte[] robotPacket1 = new byte[32];
            robotPacket1[0] = (byte)TJCommandID.EnableDisableCameraCmdID;
            robotPacket1[2] = (byte)0;
            TJDragonfly.EnqueueTXPacket(robotPacket1);
            byte[] robotPacket2 = new byte[32];
            robotPacket2[0] = (byte)TJCommandID.EnableDisablePX4CmdID;
            robotPacket2[2] = (byte)0;
            TJDragonfly.EnqueueTXPacket(robotPacket2);
            cbbPosHld.SelectedIndex = 0;
            cbbCam.SelectedIndex = 0;
            cbbPx4flow.SelectedIndex = 0;
        }

        /// <summary>
        /// Called when the throttle value is changed
        /// </summary>
        private void Throttles_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            Slider slider = sender as Slider;

            sldPidThrottle.Value = slider.Value;
            sldDFThrottle.Value = slider.Value;

            labPidThrottleValue.Content = ((int)slider.Value).ToString();
            labDFThrottleValue.Content = ((int)slider.Value).ToString();

            if (tuningTimer.IsRunning)
            {
                byte[] throttlePacket = new byte[32];
                throttlePacket[0] = (byte)TJCommandID.ThrottleCmdID;
                throttlePacket[2] = (byte)slider.Value;
                TJDragonfly.EnqueueTXPacket(throttlePacket);
            }
        }

        /// <summary>
        /// Called when the trimming values are changed
        /// </summary>
        private void Trim_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            Slider slider = sender as Slider;

            if (slider.Name.Contains("Roll"))
            {
                tbxRollTrimValue.Text = ((int)slider.Value).ToString();
                tbxDFRollTrimValue.Text = ((int)slider.Value).ToString();

                sldRollTrim.Value = slider.Value;
                sldDFRollTrim.Value = slider.Value;
            }
            else if (slider.Name.Contains("Pitch"))
            {
                tbxPitchTrimValue.Text = ((int)slider.Value).ToString();
                tbxDFPitchTrimValue.Text = ((int)slider.Value).ToString();

                sldPitchTrim.Value = slider.Value;
                sldDFPitchTrim.Value = slider.Value;
            }
            else if (slider.Name.Contains("Yaw"))
            {
                tbxYawTrimValue.Text = ((int)slider.Value).ToString();
                //tbxDFYawTrimValue.Text = ((int)slider.Value).ToString();

                yawKnob.Value = (int)slider.Value + 45;
            }

            byte[] throttlePacket = new byte[32];
            throttlePacket[0] = (byte)TJCommandID.TrimCmdID;
            throttlePacket[2] = (byte)sldRollTrim.Value;
            throttlePacket[3] = (byte)sldPitchTrim.Value;
            throttlePacket[4] = (byte)sldYawTrim.Value;
            TJDragonfly.EnqueueTXPacket(throttlePacket);
        }

        /// <summary>
        /// Called when the yaw trimming value is changed
        /// </summary>
        private void YawTrim_ValueChanged(object Sender)
        {
            int value = yawKnob.Value - 45;
            tbxDFYawTrimValue.Text = value.ToString();

            sldYawTrim.Value = value;
        }

        /// <summary>
        /// Called when the Select State drop down lists are closed
        /// </summary>
        private void cbbAltHold_DropDownClosed(object sender, EventArgs e)
        {
            ComboBox combobox = sender as ComboBox;
            if (combobox.SelectedIndex >= 0)
            {
                byte[] robotPacket = new byte[32];
                robotPacket[0] = (byte)TJCommandID.AltHoldCmdID;
                robotPacket[2] = (byte)combobox.SelectedIndex;

                TJDragonfly.EnqueueTXPacket(robotPacket);
            }
        }

        /// <summary>
        /// Called when the Position Hld (or Camera) drop down lists are closed
        /// </summary>
        private void cbbPosHld_DropDownClosed(object sender, EventArgs e)
        {
            ComboBox combobox = sender as ComboBox;

            if (combobox.SelectedIndex >= 0)
            {
                cbbPosHld.SelectedIndex = combobox.SelectedIndex;
                cbbCam.SelectedIndex = combobox.SelectedIndex;

                byte[] robotPacket = new byte[32];
                robotPacket[0] = (byte)TJCommandID.EnableDisableCameraCmdID;
                robotPacket[2] = (byte)combobox.SelectedIndex;

                if (combobox.SelectedIndex == 1)
                    btnGetFrame.IsEnabled = true;
                else
                    btnGetFrame.IsEnabled = false;

                TJDragonfly.EnqueueTXPacket(robotPacket);
            }
        }

        /// <summary>
        /// Called when the enter is pressed for inputting data value to textboxes directly
        /// </summary>
        private void Trim_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Return)
            {
                try
                {
                    TextBox textbox = sender as TextBox;
                    if (textbox.Name.Contains("Roll"))
                    {
                        sldRollTrim.Value = Convert.ToInt16(textbox.Text);
                        sldDFRollTrim.Value = Convert.ToInt16(textbox.Text);
                    }
                    else if (textbox.Name.Contains("Pitch"))
                    {
                        sldPitchTrim.Value = Convert.ToInt16(textbox.Text);
                        sldDFPitchTrim.Value = Convert.ToInt16(textbox.Text);
                    }
                    else if (textbox.Name.Contains("Yaw"))
                    {
                        sldYawTrim.Value = Convert.ToInt16(textbox.Text);
                        yawKnob.Value = Convert.ToInt16(textbox.Text) + 45;
                    }
                }
                catch (Exception)
                {
                    //throw;
                }
            }
        }

        /// <summary>
        /// Called when the gain tuning buttons are clicked
        /// </summary>
        private void Gains_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            Slider slider = sender as Slider;
            string valueString = ((int)((int)slider.Value * 1000 / 32768.0) / 1000.0).ToString(); // max value of slider.Value is 32767, to map it to the range of 0 to 0.999
            Int16 gainValue = (Int16)slider.Value;

            int[] index = { -1, -1, -1 };
            index = FindControl(slider.Name);

            if (index[0] >= 0 && index[1] >= 0 && index[2] >= 0)
            {
                // New version
                TJPIDControlGains gain = gains[index[0]][index[1]];

                switch (index[2])
                {
                    case kpIndex:
                        gain.Kp = gainValue; break;
                    case kiIndex:
                        gain.Ki = gainValue; break;
                    case kdIndex:
                        gain.Kd = gainValue; break;
                    default: break;
                }

                if ((bool)cbxLockRollPitch.IsChecked && (index[1] == 0 || index[1] == 1))
                {
                    for (int i = 0; i < 2; i++)
                    {
                        TJDragonfly.SetGains(index[0], i, gains[index[0]][i]);

                        System.Windows.Controls.TextBox tb = gainTextboxes[index[0]][i][index[2]];
                        System.Windows.Controls.Slider sb = gainSliders[index[0]][i][index[2]];
                        sb.Value = slider.Value; // 0 to 32767
                        tb.Text = valueString; // 0 to 0.999
                    }
                }
                else
                {
                    TJDragonfly.SetGains(index[0], index[1], gains[index[0]][index[1]]);

                    System.Windows.Controls.TextBox tb = gainTextboxes[index[0]][index[1]][index[2]];
                    System.Windows.Controls.Slider sb = gainSliders[index[0]][index[1]][index[2]];
                    sb.Value = slider.Value;
                    tb.Text = valueString;
                }

                // Old version
                //int directionIndex = index[1];
                //int gainIndex = index[2];

                //TJPIDControlGains gain = gains[0][directionIndex];

                //System.Windows.Controls.Slider sb = gainSliders[0][directionIndex][gainIndex];
                //System.Windows.Controls.TextBox tbx;

                //if ((directionIndex == rollIndex || directionIndex == pitchIndex) && (bool)cbxLockRollPitch.IsChecked) // Quick & dirty code to lock both roll/pitch pid gain changes
                //{
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[0][rollIndex][gainIndex];
                //    tbx.Text = valueString;

                //    sb = gainSliders[0][pitchIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[0][pitchIndex][gainIndex];
                //    tbx.Text = valueString;

                //    // Change value of other sliders
                //    sb = gainSliders[1][rollIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[1][rollIndex][gainIndex];
                //    tbx.Text = valueString;
                //    sb = gainSliders[1][pitchIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[1][pitchIndex][gainIndex];
                //    tbx.Text = valueString;

                //    sb = gainSliders[2][rollIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[2][rollIndex][gainIndex];
                //    tbx.Text = valueString;
                //    sb = gainSliders[2][pitchIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[2][pitchIndex][gainIndex];
                //    tbx.Text = valueString;
                //}
                //else
                //{
                //    sb = gainSliders[0][directionIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[0][directionIndex][gainIndex];
                //    tbx.Text = valueString;

                //    sb = gainSliders[1][directionIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[1][directionIndex][gainIndex];
                //    tbx.Text = valueString;

                //    sb = gainSliders[2][directionIndex][gainIndex];
                //    sb.Value = slider.Value;
                //    tbx = gainTextboxes[2][directionIndex][gainIndex];
                //    tbx.Text = valueString;
                //}

                //switch (gainIndex)
                //{
                //    case kpIndex:
                //        gain.Kp = gainValue;
                //        break;
                //    case kiIndex:
                //        gain.Ki = gainValue;
                //        break;
                //    case kdIndex:
                //        gain.Kd = gainValue;
                //        break;
                //    default: break;
                //}

                //switch (directionIndex)
                //{
                //    case rollIndex:
                //        if ((bool)cbxLockRollPitch.IsChecked)
                //            TJDragonfly.SetGains(gains[0][rollIndex], gains[0][rollIndex], null);
                //        else
                //            TJDragonfly.SetGains(gains[0][rollIndex], null, null);

                //        break;
                //    case pitchIndex:
                //        if ((bool)cbxLockRollPitch.IsChecked)
                //            TJDragonfly.SetGains(gains[0][pitchIndex], gains[0][pitchIndex], null);
                //        else
                //            TJDragonfly.SetGains(null, gains[0][pitchIndex], null);

                //        break;
                //    case yawIndex:
                //        TJDragonfly.SetGains(null, null, gains[0][yawIndex]);
                //        break;
                //}
            }
        }

        /// <summary>
        /// Called when the gain tuning up buttons are clicked
        /// </summary>
        private void Gains_UpClick(object sender, RoutedEventArgs e)
        {
            Button button = sender as Button;

            int[] index = { -1, -1, -1 };
            index = FindControl(button.Name);

            if (index[0] >= 0 && index[1] >= 0 && index[2] >= 0)
            {
                // New version
                System.Windows.Controls.Slider sb = gainSliders[index[0]][index[1]][index[2]];
                sb.Value += 200;

                // Old version
                //System.Windows.Controls.Slider sb = gainSliders[0][index[1]][index[2]];
                //sb.Value += 200;
            }
        }

        /// <summary>
        /// Called when the gain tuning down buttons are clicked
        /// </summary>
        private void Gains_DownClick(object sender, RoutedEventArgs e)
        {
            Button button = sender as Button;

            int[] index = { -1, -1, -1 };
            index = FindControl(button.Name);

            if (index[0] >= 0 && index[1] >= 0 && index[2] >= 0)
            {
                // New version
                System.Windows.Controls.Slider sb = gainSliders[index[0]][index[1]][index[2]];
                sb.Value -= 200;

                // Old version
                //System.Windows.Controls.Slider sb = gainSliders[0][index[1]][index[2]];
                //sb.Value -= 200;
            }
        }

        /// <summary>
        /// Enter the gains directly from the textboxes
        /// </summary>
        private void Gains_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Return)
            {
                TextBox textbox = sender as TextBox;
                double value = 0;
                int[] index = { -1, -1, -1 };
                index = FindControl(textbox.Name);

                try
                {
                    value = Convert.ToDouble(textbox.Text);
                }
                catch (Exception)
                {
                    //throw;
                }

                if (value >= 0 && value <= 1)
                {
                    // New version
                    System.Windows.Controls.Slider sb = gainSliders[index[0]][index[1]][index[2]];

                    // Old version
                    //System.Windows.Controls.Slider sb = gainSliders[0][index[1]][index[2]];

                    sb.Value = value * 32768.0;
                }
            }
        }

        /// <summary>
        /// Called when the texts of reference time is changed
        /// </summary>
        private void TimerRef_TextChanged(object sender, TextChangedEventArgs e)
        {
            TextBox textbox = sender as TextBox;

            tbxPidTimerRef.Text = textbox.Text;
            tbxDFTimerRef.Text = textbox.Text;
        }

        private int[] FindControl(string s)
        {
            int[] index = { -1, -1, -1 }; // Control index (pos, agl, agr), direction index (roll, pitch, yaw), gain index (kp, ki, kd)

            //Control index (pos, agl, agr)
            if (s.Contains("Pos"))
            {
                index[0] = 0;
            }
            else if (s.Contains("Agl"))
            {
                index[0] = 1;
            }
            else if (s.Contains("Agr"))
            {
                index[0] = 2;
            }

            // Direction index (roll, pitch, yaw)
            if (s.Contains("Roll"))
            {
                index[1] = 0;
            }
            else if (s.Contains("Pitch"))
            {
                index[1] = 1;
            }
            else if (s.Contains("Yaw"))
            {
                index[1] = 2;
            }

            // Gain index (kp, ki, kd)
            if (s.Contains("Kp"))
            {
                index[2] = 0;
            }
            else if (s.Contains("Ki"))
            {
                index[2] = 1;
            }
            else if (s.Contains("Kd"))
            {
                index[2] = 2;
            }

            return index;
        }

        // =================================================================== DF Tuning Tab ===================================================================
        /// <summary>
        /// Called when the update value button is clicked
        /// </summary>
        private void btnUpdateTuningValues_Click(object sender, RoutedEventArgs e)
        {
            Button button = sender as Button;
            button.IsEnabled = false;

            // Update throttle and trim values
            byte[] trimPacket = new byte[32];
            trimPacket[0] = (byte)TJCommandID.TrimCmdID;
            trimPacket[2] = (byte)sldRollTrim.Value;
            trimPacket[3] = (byte)sldPitchTrim.Value;
            trimPacket[4] = (byte)sldYawTrim.Value;

            TJDragonfly.EnqueueTXPacket(trimPacket);

            // Update PID tuning values
            // New version

            // Old version
            TJDragonfly.SetGains(gains[0][rollIndex], gains[0][pitchIndex], gains[0][yawIndex]);

            // Update servo nominal and saturation values for Dragonfly
            byte[] dfPacket = new byte[32];
            dfPacket[0] = (byte)TJCommandID.DFNomSatCmdID; // 0x03

            if (tuningTimer.IsRunning)
                dfPacket[2] = (byte)this.sldDFMainMotor.Value;
            else
                dfPacket[2] = 0;

            dfPacket[3] = (byte)this.sldDFServoNom.Value;
            dfPacket[4] = (byte)this.sldDFServoSat.Value;
            dfPacket[5] = (byte)this.sldDFPitchSat.Value;
            dfPacket[6] = (byte)this.sldDFYawSat.Value;
            TJDragonfly.EnqueueTXPacket(dfPacket);

            byte[] sTrimPacket = new byte[32];
            sTrimPacket[0] = (byte)TJCommandID.DFServoTrimCmdID; // 0x21
            sTrimPacket[2] = (byte)this.sldDFServoTrimFL.Value;
            sTrimPacket[3] = (byte)(-this.sldDFServoTrimFR.Value); // Send reverse value
            sTrimPacket[4] = (byte)(-this.sldDFServoTrimBL.Value); // Send reverse value
            sTrimPacket[5] = (byte)this.sldDFServoTrimBR.Value;
            TJDragonfly.EnqueueTXPacket(sTrimPacket);

            // Update wing amplitude
            UpdateWingAmpMin();
            UpdateWingAmpMax();

            button.IsEnabled = true;
        }

        /// <summary>
        /// Called when the save tuning button is clicked
        /// </summary>
        private void btnSaveTuningSetting_Click(object sender, RoutedEventArgs e)
        {
            // Create the XmlDocument.
            XmlDocument doc = new XmlDocument();
            doc.LoadXml("<item><name>Parameters</name></item>");

            // Add elements
            //XmlElement newElement = doc.CreateElement(sldPidThrottle.Name);
            //newElement.InnerText = sldPidThrottle.Value.ToString();
            //doc.DocumentElement.AppendChild(newElement);

            XmlElement newElement;
            foreach (var s1 in gainSliders)
            {
                foreach (var s2 in s1)
                {
                    foreach (var s3 in s2)
                    {
                        newElement = doc.CreateElement(s3.Name);
                        newElement.InnerText = s3.Value.ToString();
                        doc.DocumentElement.AppendChild(newElement);
                    }
                }
            }

            Slider[] trimSliders = new Slider[]
            {
                sldRollTrim,
                sldPitchTrim,
                sldYawTrim,
            };

            foreach (var slider in trimSliders)
            {
                newElement = doc.CreateElement(slider.Name);
                newElement.InnerText = slider.Value.ToString();
                doc.DocumentElement.AppendChild(newElement);
            }

            // Dragonfly parameters
            Slider[] DFSliders = new Slider[]
            {
                //sldDFMainMotor,
                sldDFServoNom,
                sldDFServoSat,
                sldDFPitchSat,
                sldDFYawSat,
                sldDFServoTrimFL,
                sldDFServoTrimFR,
                sldDFServoTrimBL,
                sldDFServoTrimBR,
            };

            foreach (var slider in DFSliders)
            {
                newElement = doc.CreateElement(slider.Name);
                newElement.InnerText = slider.Value.ToString();
                doc.DocumentElement.AppendChild(newElement);
            }

            TextBox[] DFTextboxes = new TextBox[]
            {
                tbxDFWingAmpFLMin,
                tbxDFWingAmpFLMax,
                tbxDFWingAmpFRMin,
                tbxDFWingAmpFRMax,
                tbxDFWingAmpBLMin,
                tbxDFWingAmpBLMax,
                tbxDFWingAmpBRMin,
                tbxDFWingAmpBRMax,
            };

            foreach (var textbox in DFTextboxes)
            {
                newElement = doc.CreateElement(textbox.Name);
                newElement.InnerText = textbox.Text;
                doc.DocumentElement.AppendChild(newElement);
            }

            System.Windows.Forms.SaveFileDialog dialog = new System.Windows.Forms.SaveFileDialog();
            dialog.Filter = "XML-File | *.xml";
            dialog.Title = "Save the setting";
            dialog.ShowDialog();

            if (dialog.FileName != "")
            {
                doc.Save(dialog.FileName);
            }
        }

        /// <summary>
        /// Called when the load tuning button is clicked
        /// </summary>
        private void btnLoadTuningSetting_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.Forms.OpenFileDialog dialog = new System.Windows.Forms.OpenFileDialog();
            dialog.Filter = "XML-File | *.xml";
            dialog.Title = "Load a setting";
            dialog.ShowDialog();

            if (dialog.FileName != "")
            {
                XmlDocument xml = new XmlDocument();
                xml.Load(dialog.FileName);

                //XmlNodeList elemList = xml.GetElementsByTagName("sldPidThrottle");
                //for (int i = 0; i < elemList.Count; i++)
                //    sldPidThrottle.Value = Convert.ToDouble(elemList[i].InnerText);

                XmlNodeList elemList;
                Slider[] trimSliders = new Slider[]
                {
                    sldRollTrim,
                    sldPitchTrim,
                    sldYawTrim,
                };

                foreach (var slider in trimSliders)
                {
                    elemList = xml.GetElementsByTagName(slider.Name);
                    for (int i = 0; i < elemList.Count; i++)
                        slider.Value = Convert.ToDouble(elemList[i].InnerText);
                }

                foreach (var s1 in gainSliders)
                {
                    foreach (var s2 in s1)
                    {
                        foreach (var s3 in s2)
                        {
                            elemList = xml.GetElementsByTagName(s3.Name);
                            for (int i = 0; i < elemList.Count; i++)
                                s3.Value = Convert.ToDouble(elemList[i].InnerText);
                        }
                    }
                }

                // Dragonfly parameters
                Slider[] DFSliders = new Slider[]
                {
                    //sldDFMainMotor,
                    sldDFServoNom,
                    sldDFServoSat,
                    sldDFPitchSat,
                    sldDFYawSat,
                    sldDFServoTrimFL,
                    sldDFServoTrimFR,
                    sldDFServoTrimBL,
                    sldDFServoTrimBR,
                };

                foreach (var slider in DFSliders)
                {
                    elemList = xml.GetElementsByTagName(slider.Name);
                    for (int i = 0; i < elemList.Count; i++)
                        slider.Value = Convert.ToDouble(elemList[i].InnerText);
                }

                TextBox[] DFTextboxes = new TextBox[]
                {
                    tbxDFWingAmpFLMin,
                    tbxDFWingAmpFLMax,
                    tbxDFWingAmpFRMin,
                    tbxDFWingAmpFRMax,
                    tbxDFWingAmpBLMin,
                    tbxDFWingAmpBLMax,
                    tbxDFWingAmpBRMin,
                    tbxDFWingAmpBRMax,
                };

                foreach (var textbox in DFTextboxes)
                {
                    elemList = xml.GetElementsByTagName(textbox.Name);
                    for (int i = 0; i < elemList.Count; i++)
                        textbox.Text = elemList[i].InnerText;
                }

                // Update minimum values
                UpdateWingAmpMin();

                // Update maximum values
                UpdateWingAmpMax();
            }
        }

        /// <summary>
        /// Called when the robot initialization drop down lists are closed
        /// </summary>
        private void cbbRobotInitialization_DropDownClosed(object sender, EventArgs e)
        {
            ComboBox combobox = sender as ComboBox;
            if (combobox.SelectedIndex >= 0)
            {
                int neg = -1;
                byte[] robotPacket = new byte[32];
                robotPacket[0] = (byte)TJCommandID.SetAddressCmdID;
                robotPacket[2] = (byte)neg;
                robotPacket[3] = (byte)neg;
                robotPacket[4] = (byte)neg;
                robotPacket[5] = (byte)neg;

                if (combobox.Name.Contains("WingSeq"))
                    robotPacket[3] = (byte)combobox.SelectedIndex;
                else if (combobox.Name.Contains("RCCalibration"))
                    robotPacket[4] = (byte)combobox.SelectedIndex;
                else if (combobox.Name.Contains("RCFilter"))
                    robotPacket[5] = (byte)combobox.SelectedIndex;

                TJDragonfly.EnqueueTXPacket(robotPacket);
            }
        }

        /// <summary>
        /// Called when the DF robot's tuning values are changed
        /// </summary>
        private void DFAddional_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (sldDFMainMotor == null || sldPidMainMotor == null || sldDFServoNom == null || sldDFServoSat == null || sldDFPitchSat == null || sldDFYawSat == null)
                return;

            Slider slider = sender as Slider;
            if (slider.Name.Contains("MainMotor"))
            {
                labPidMainMotorValue.Content = ((int)slider.Value).ToString();
                labDFMainMotorValue.Content = ((int)slider.Value).ToString();

                sldPidMainMotor.Value = slider.Value;
                sldDFMainMotor.Value = slider.Value;
            }

            labDFServoNomValue.Content = ((int)this.sldDFServoNom.Value).ToString();
            tbxDFServoSatValue.Text = ((int)this.sldDFServoSat.Value).ToString();
            tbxDFPitchSatValue.Text = ((int)this.sldDFPitchSat.Value).ToString();
            tbxDFYawSatValue.Text = ((int)this.sldDFYawSat.Value).ToString();

            byte[] dfPacket = new byte[32];
            dfPacket[0] = (byte)TJCommandID.DFNomSatCmdID; // 0x20

            if (tuningTimer.IsRunning)
                dfPacket[2] = (byte)this.sldDFMainMotor.Value;
            else
                dfPacket[2] = 0;

            dfPacket[3] = (byte)this.sldDFServoNom.Value;
            dfPacket[4] = (byte)this.sldDFServoSat.Value;
            dfPacket[5] = (byte)this.sldDFPitchSat.Value;
            dfPacket[6] = (byte)this.sldDFYawSat.Value;
            TJDragonfly.EnqueueTXPacket(dfPacket);
        }

        /// <summary>
        /// Enter the saturation values directly
        /// </summary>
        private void Saturation_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Return)
            {
                try
                {
                    sldDFServoSat.Value = Convert.ToInt16(tbxDFServoSatValue.Text);
                    sldDFPitchSat.Value = Convert.ToInt16(tbxDFPitchSatValue.Text);
                    sldDFYawSat.Value = Convert.ToInt16(tbxDFYawSatValue.Text);
                }
                catch (Exception)
                {
                    //throw;
                }
            }
        }

        /// <summary>
        /// Reset the saturation values
        /// </summary>
        private void btnDFSaturationReset_Click(object sender, RoutedEventArgs e)
        {
            sldDFServoSat.Value = 0;
            sldDFPitchSat.Value = 0;
            sldDFYawSat.Value = 0;
        }

        private void WingAmp_UpClick(object sender, RoutedEventArgs e)
        {
            Button button = sender as Button;

            try
            {
                short FLMin = Convert.ToInt16(tbxDFWingAmpFLMin.Text);
                short FRMin = Convert.ToInt16(tbxDFWingAmpFRMin.Text);
                short BLMin = Convert.ToInt16(tbxDFWingAmpBLMin.Text);
                short BRMin = Convert.ToInt16(tbxDFWingAmpBRMin.Text);

                short FLMax = Convert.ToInt16(tbxDFWingAmpFLMax.Text);
                short FRMax = Convert.ToInt16(tbxDFWingAmpFRMax.Text);
                short BLMax = Convert.ToInt16(tbxDFWingAmpBLMax.Text);
                short BRMax = Convert.ToInt16(tbxDFWingAmpBRMax.Text);

                if (button.Name.Contains("FL"))
                {
                    tbxDFWingAmpFLMin.Text = (FLMin + 50).ToString();
                    tbxDFWingAmpFLMax.Text = (FLMax + 50).ToString();
                }
                else if (button.Name.Contains("FR"))
                {
                    tbxDFWingAmpFRMin.Text = (FRMin + 50).ToString();
                    tbxDFWingAmpFRMax.Text = (FRMax + 50).ToString();
                }
                else if (button.Name.Contains("BL"))
                {
                    tbxDFWingAmpBLMin.Text = (BLMin + 50).ToString();
                    tbxDFWingAmpBLMax.Text = (BLMax + 50).ToString();
                }
                else if (button.Name.Contains("BR"))
                {
                    tbxDFWingAmpBRMin.Text = (BRMin + 50).ToString();
                    tbxDFWingAmpBRMax.Text = (BRMax + 50).ToString();
                }

                UpdateWingAmpMin();
                UpdateWingAmpMax();
            }
            catch (Exception)
            {
                //throw;
            }
        }

        /// <summary>
        /// Decrease the wing amplitude
        /// </summary>
        private void WingAmp_DownClick(object sender, RoutedEventArgs e)
        {
            Button button = sender as Button;

            try
            {
                short FLMin = Convert.ToInt16(tbxDFWingAmpFLMin.Text);
                short FRMin = Convert.ToInt16(tbxDFWingAmpFRMin.Text);
                short BLMin = Convert.ToInt16(tbxDFWingAmpBLMin.Text);
                short BRMin = Convert.ToInt16(tbxDFWingAmpBRMin.Text);

                short FLMax = Convert.ToInt16(tbxDFWingAmpFLMax.Text);
                short FRMax = Convert.ToInt16(tbxDFWingAmpFRMax.Text);
                short BLMax = Convert.ToInt16(tbxDFWingAmpBLMax.Text);
                short BRMax = Convert.ToInt16(tbxDFWingAmpBRMax.Text);

                if (button.Name.Contains("FL"))
                {
                    tbxDFWingAmpFLMin.Text = (FLMin - 50).ToString();
                    tbxDFWingAmpFLMax.Text = (FLMax - 50).ToString();
                }
                else if (button.Name.Contains("FR"))
                {
                    tbxDFWingAmpFRMin.Text = (FRMin - 50).ToString();
                    tbxDFWingAmpFRMax.Text = (FRMax - 50).ToString();
                }
                else if (button.Name.Contains("BL"))
                {
                    tbxDFWingAmpBLMin.Text = (BLMin - 50).ToString();
                    tbxDFWingAmpBLMax.Text = (BLMax - 50).ToString();
                }
                else if (button.Name.Contains("BR"))
                {
                    tbxDFWingAmpBRMin.Text = (BRMin - 50).ToString();
                    tbxDFWingAmpBRMax.Text = (BRMax - 50).ToString();
                }

                UpdateWingAmpMin();
                UpdateWingAmpMax();
            }
            catch (Exception)
            {
                //throw;
            }
        }

        /// <summary>
        /// Decrease the minimum and maximum of wing amplitude values
        /// </summary>
        private void WingAmpMinMax_KeyDown(object sender, KeyEventArgs e)
        {

            if (e.Key == Key.Return)
            {
                try
                {
                    TextBox textbox = sender as TextBox;

                    short FLMin = Convert.ToInt16(tbxDFWingAmpFLMin.Text);
                    short FRMin = Convert.ToInt16(tbxDFWingAmpFRMin.Text);
                    short BLMin = Convert.ToInt16(tbxDFWingAmpBLMin.Text);
                    short BRMin = Convert.ToInt16(tbxDFWingAmpBRMin.Text);

                    short FLMax = Convert.ToInt16(tbxDFWingAmpFLMax.Text);
                    short FRMax = Convert.ToInt16(tbxDFWingAmpFRMax.Text);
                    short BLMax = Convert.ToInt16(tbxDFWingAmpBLMax.Text);
                    short BRMax = Convert.ToInt16(tbxDFWingAmpBRMax.Text);

                    if (textbox.Name.Contains("Min"))
                    {
                        if (textbox.Name.Contains("FL"))
                            tbxDFWingAmpFLMax.Text = (FLMin + 2100).ToString();
                        else if (textbox.Name.Contains("FR"))
                            tbxDFWingAmpFRMax.Text = (FRMin - 2100).ToString();
                        else if (textbox.Name.Contains("BL"))
                            tbxDFWingAmpBLMax.Text = (BLMin - 2100).ToString();
                        else if (textbox.Name.Contains("BR"))
                            tbxDFWingAmpBRMax.Text = (BRMin + 2100).ToString();
                    }
                    else if (textbox.Name.Contains("Max"))
                    {
                        if (textbox.Name.Contains("FL"))
                            tbxDFWingAmpFLMin.Text = (FLMax - 2100).ToString();
                        else if (textbox.Name.Contains("FR"))
                            tbxDFWingAmpFRMin.Text = (FRMax + 2100).ToString();
                        else if (textbox.Name.Contains("BL"))
                            tbxDFWingAmpBLMin.Text = (BLMax + 2100).ToString();
                        else if (textbox.Name.Contains("BR"))
                            tbxDFWingAmpBRMin.Text = (BRMax - 2100).ToString();
                    }

                    UpdateWingAmpMin();
                    UpdateWingAmpMax();
                }
                catch (Exception)
                {
                    //throw;
                }
            }
        }

        /// <summary>
        /// Reset the wing amplitude
        /// </summary>
        private void btnDFWingAmpReset_Click(object sender, RoutedEventArgs e)
        {
            tbxDFWingAmpFLMin.Text = "1700";
            tbxDFWingAmpFRMin.Text = "3800";
            tbxDFWingAmpBLMin.Text = "3800";
            tbxDFWingAmpBRMin.Text = "1700";

            tbxDFWingAmpFLMax.Text = "3800";
            tbxDFWingAmpFRMax.Text = "1700";
            tbxDFWingAmpBLMax.Text = "1700";
            tbxDFWingAmpBRMax.Text = "3800";

            UpdateWingAmpMin();
            UpdateWingAmpMax();
        }

        /// <summary>
        /// Reset the trimming value of DF robot
        /// </summary>
        private void btnDFTrimReset_Click(object sender, RoutedEventArgs e)
        {
            sldRollTrim.Value = 0;
            sldPitchTrim.Value = 0;
            sldYawTrim.Value = 0;
        }

        /// <summary>
        /// Reset the wing amplitude
        /// </summary>
        void UpdateWingAmpMin()
        {
            short FLMin = Convert.ToInt16(tbxDFWingAmpFLMin.Text);
            short FRMin = Convert.ToInt16(tbxDFWingAmpFRMin.Text);
            short BLMin = Convert.ToInt16(tbxDFWingAmpBLMin.Text);
            short BRMin = Convert.ToInt16(tbxDFWingAmpBRMin.Text);

            byte[] minPacket = new byte[32];
            minPacket[0] = (byte)TJCommandID.DFPWMMinCmdID; // 0x22

            minPacket[2] = (byte)(FLMin & 0xFF);
            minPacket[3] = (byte)((FLMin >> 8) & 0xFF);
            minPacket[4] = (byte)(FRMin & 0xFF);
            minPacket[5] = (byte)((FRMin >> 8) & 0xFF);
            minPacket[6] = (byte)(BLMin & 0xFF);
            minPacket[7] = (byte)((BLMin >> 8) & 0xFF);
            minPacket[8] = (byte)(BRMin & 0xFF);
            minPacket[9] = (byte)((BRMin >> 8) & 0xFF);

            TJDragonfly.EnqueueTXPacket(minPacket);
        }

        /// <summary>
        /// Increase the maximum of the wing amplitude
        /// </summary>
        void UpdateWingAmpMax()
        {
            short FLMax = Convert.ToInt16(tbxDFWingAmpFLMax.Text);
            short FRMax = Convert.ToInt16(tbxDFWingAmpFRMax.Text);
            short BLMax = Convert.ToInt16(tbxDFWingAmpBLMax.Text);
            short BRMax = Convert.ToInt16(tbxDFWingAmpBRMax.Text);

            byte[] maxPacket = new byte[32];
            maxPacket[0] = (byte)TJCommandID.DFPWMMaxCmdID; // 0x23

            maxPacket[2] = (byte)(FLMax & 0xFF);
            maxPacket[3] = (byte)((FLMax >> 8) & 0xFF);
            maxPacket[4] = (byte)(FRMax & 0xFF);
            maxPacket[5] = (byte)((FRMax >> 8) & 0xFF);
            maxPacket[6] = (byte)(BLMax & 0xFF);
            maxPacket[7] = (byte)((BLMax >> 8) & 0xFF);
            maxPacket[8] = (byte)(BRMax & 0xFF);
            maxPacket[9] = (byte)((BRMax >> 8) & 0xFF);

            TJDragonfly.EnqueueTXPacket(maxPacket);
        }

        /// <summary>
        /// Called when value of servo trimming is changed
        /// </summary>
        private void ServoTrim_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            tbxDFServoTrimFLValue.Text = ((int)sldDFServoTrimFL.Value).ToString();
            tbxDFServoTrimFRValue.Text = ((int)sldDFServoTrimFR.Value).ToString();
            tbxDFServoTrimBLValue.Text = ((int)sldDFServoTrimBL.Value).ToString();
            tbxDFServoTrimBRValue.Text = ((int)sldDFServoTrimBR.Value).ToString();

            byte[] dfPacket = new byte[32];
            dfPacket[0] = (byte)TJCommandID.DFServoTrimCmdID; // 0x21
            dfPacket[2] = (byte)this.sldDFServoTrimFL.Value;
            dfPacket[3] = (byte)(-this.sldDFServoTrimFR.Value); // Send reverse value
            dfPacket[4] = (byte)(-this.sldDFServoTrimBL.Value); // Send reverse value
            dfPacket[5] = (byte)this.sldDFServoTrimBR.Value;
            TJDragonfly.EnqueueTXPacket(dfPacket);
        }

        /// <summary>
        /// Called when servo trimming up botton is pressed
        /// </summary>
        private void ServoTrim_UpClick(object sender, RoutedEventArgs e)
        {
            Button button = sender as Button;

            if (button.Name.Contains("FL"))
                sldDFServoTrimFL.Value += 1;
            else if (button.Name.Contains("FR"))
                sldDFServoTrimFR.Value += 1;
            else if (button.Name.Contains("BL"))
                sldDFServoTrimBL.Value += 1;
            else if (button.Name.Contains("BR"))
                sldDFServoTrimBR.Value += 1;
        }

        /// <summary>
        /// Called when servo trimming down botton is pressed
        /// </summary>
        private void ServoTrim_DownClick(object sender, RoutedEventArgs e)
        {
            Button button = sender as Button;

            if (button.Name.Contains("FL"))
                sldDFServoTrimFL.Value -= 1;
            else if (button.Name.Contains("FR"))
                sldDFServoTrimFR.Value -= 1;
            else if (button.Name.Contains("BL"))
                sldDFServoTrimBL.Value -= 1;
            else if (button.Name.Contains("BR"))
                sldDFServoTrimBR.Value -= 1;
        }

        /// <summary>
        /// Called when servo trimming value is entered directly
        /// </summary>
        private void ServoTrim_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Return)
            {
                TextBox textbox = sender as TextBox;
                double value = 0;

                try
                {
                    value = Convert.ToInt32(textbox.Text);
                }
                catch (Exception)
                {
                    //throw;
                }

                if (value >= -100 && value <= 100)
                {
                    if (textbox.Name.Contains("FL"))
                        sldDFServoTrimFL.Value = value;
                    else if (textbox.Name.Contains("FR"))
                        sldDFServoTrimFR.Value = value;
                    else if (textbox.Name.Contains("BL"))
                        sldDFServoTrimBL.Value = value;
                    else if (textbox.Name.Contains("BR"))
                        sldDFServoTrimBR.Value = value;
                }
            }
        }

        /// <summary>
        /// Reset the trimming value of DF servo
        /// </summary>
        private void btnDFServoTrimReset_Click(object sender, RoutedEventArgs e)
        {
            sldDFServoTrimFL.Value = 0;
            sldDFServoTrimFR.Value = 0;
            sldDFServoTrimBL.Value = 0;
            sldDFServoTrimBR.Value = 0;
        }

        // =================================================================== Optical Flow Control Tab ===================================================================
        ///// <summary>
        ///// Called when the camera button is clicked, enable the camera
        ///// </summary>
        //private void btnCamera_Click(object sender, RoutedEventArgs e)
        //{
        //    if (btnCamera == null || btnGetFrame == null)
        //        return;

        //    TJCommand cmd = new TJCommand(TJCommandID.ToggleCameraCmdID);
        //    TJDragonfly.EnqueueCommand(cmd);
        //    if (btnCamera.Content.ToString() == "Enable Camera")
        //    {
        //        btnCamera.Content = "Disable Camera";
        //        btnGetFrame.IsEnabled = true;
        //    }
        //    else
        //    {
        //        btnCamera.Content = "Enable Camera";
        //        btnGetFrame.IsEnabled = false;
        //    }
        //}

        /// <summary>
        /// Called when the get frame button is clicked
        /// </summary>
        private void btnGetFrame_Click(object sender, RoutedEventArgs e)
        {
            //if (btnCamera == null || btnGetFrame == null)
            //    return;

            if (btnGetFrame.IsEnabled)
            {
                TJCommand cmd = new TJCommand(TJCommandID.GetFrameCmdID);
                TJDragonfly.EnqueueCommand(cmd);
            }
        }

        /// <summary>
        /// This method responds to changes in the resolution trackbars and updates the displayed image accordingly.
        /// </summary>
        private void sldCamImageWidth_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (cbxLockCamImageRatio != null)
            {
                if ((bool)cbxLockCamImageRatio.IsChecked)
                {
                    double buffer = (double)(sldCamImageHeight.Maximum + 1) - (double)(sldCamImageWidth.Value) / AspectRatio;
                    if ((buffer >= (double)sldCamImageHeight.Minimum) && (buffer <= (double)sldCamImageHeight.Maximum))
                    {
                        sldCamImageHeight.Value = (int)buffer;
                    }
                    else if ((buffer < (double)sldCamImageHeight.Minimum) || (buffer > (double)sldCamImageHeight.Maximum))
                    {
                        sldCamImageWidth.Value = (int)(AspectRatio * (double)(sldCamImageHeight.Maximum - sldCamImageHeight.Value + 1));
                    }
                }

                labCamImageSize.Content = ((int)sldCamImageWidth.Value).ToString() + " x " + ((int)(sldCamImageHeight.Maximum - sldCamImageHeight.Value + 1)).ToString();
                resizeImage();
            }
        }

        private void sldCamImageHeight_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (cbxLockCamImageRatio != null)
            {
                if ((bool)cbxLockCamImageRatio.IsChecked)
                {
                    double buffer = AspectRatio * (double)(sldCamImageHeight.Maximum - sldCamImageHeight.Value + 1);
                    if ((buffer <= (double)sldCamImageWidth.Maximum) && (buffer >= (double)sldCamImageWidth.Minimum))
                    {
                        sldCamImageWidth.Value = (int)buffer;
                    }
                    else if ((buffer < (double)sldCamImageWidth.Minimum) || (buffer > (double)sldCamImageWidth.Maximum))
                    {
                        sldCamImageHeight.Value = sldCamImageHeight.Maximum + 1 - (int)((double)(sldCamImageWidth.Value) / AspectRatio);
                    }
                }

                labCamImageSize.Content = sldCamImageWidth.Value.ToString() + " x " + (sldCamImageHeight.Maximum - sldCamImageHeight.Value + 1).ToString();
                resizeImage();
            }
        }

        /// <summary>
        /// This method resizes the image based on the current trackbar resolution settings
        /// </summary>
        void resizeImage()
        {
            Bitmap ImageToResize = KeepOriginal;
            if (BitmapList.Count > 0)
            {
                if (KeepOriginal == null)
                {
                    ImageToResize = BitmapList.Last.Value;
                    KeepOriginal = BitmapList.Last.Value;
                }
                var brush = new SolidBrush(Color.White);
                var oldWidth = ImageToResize.Width;
                var oldHeight = ImageToResize.Width;
                var newWidth = sldCamImageWidth.Value;
                var newHeight = sldCamImageHeight.Maximum - sldCamImageHeight.Value + 1;
                float scaleWidth = (float)newWidth / (float)oldWidth;
                float scaleHeight = (float)newHeight / (float)oldHeight;

                Bitmap bmp = new Bitmap(ImageToResize, (int)newWidth, (int)newHeight);
                bmp.Palette = ImageToResize.Palette;

                BitmapList.RemoveLast();
                BitmapList.AddLast(bmp);

                this.Dispatcher.BeginInvoke((Action)delegate()
                {
                    pictureBoxCam.Width = BitmapList.Last.Value.Width;
                    pictureBoxCam.Image = BitmapList.Last.Value;
                });
            }
        }

        /// <summary>
        /// Respond to OFC options change
        /// </summary>
        private void OpticalFlow_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            byte[] ofcPacket = new byte[32];
            ofcPacket[0] = (byte)TJCommandID.CameraSettingCmdID; // 0x92

            if (cbbCamera.SelectedIndex >= 0)
                ofcPacket[2] = (byte)(cbbCamera.SelectedIndex + 1);

            if (cbbCameraOption.SelectedIndex >= 0)
                ofcPacket[3] = (byte)(cbbCameraOption.SelectedIndex + 1);

            if (cbbCameraFov.SelectedIndex >= 0)
                ofcPacket[4] = Convert.ToByte(((ComboBoxItem)cbbCameraFov.SelectedItem).Content);

            TJDragonfly.EnqueueTXPacket(ofcPacket);
        }

        /// <summary>
        /// Called when save OFC setting button is clicked
        /// </summary>
        private void btnSaveOpticalSetting_Click(object sender, RoutedEventArgs e)
        {
            // Create the XmlDocument.
            XmlDocument doc = new XmlDocument();
            doc.LoadXml("<item><name>Parameters</name></item>");

            // Add elements
            //XmlElement newElement = doc.CreateElement(sldPidThrottle.Name);
            //newElement.InnerText = sldPidThrottle.Value.ToString();
            //doc.DocumentElement.AppendChild(newElement);

            System.Windows.Forms.SaveFileDialog dialog = new System.Windows.Forms.SaveFileDialog();
            dialog.Filter = "XML-File | *.xml";
            dialog.Title = "Save the setting";
            dialog.ShowDialog();

            if (dialog.FileName != "")
            {
                doc.Save(dialog.FileName);
            }
        }

        /// <summary>
        /// Called when load OFC setting button is clicked
        /// </summary>
        private void btnLoadOpticalSetting_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.Forms.OpenFileDialog dialog = new System.Windows.Forms.OpenFileDialog();
            dialog.Filter = "XML-File | *.xml";
            dialog.Title = "Load a setting";
            dialog.ShowDialog();

            if (dialog.FileName != "")
            {
                XmlDocument xml = new XmlDocument();
                xml.Load(dialog.FileName);

                //XmlNodeList elemList = xml.GetElementsByTagName("sldPidThrottle");
                //for (int i = 0; i < elemList.Count; i++)
                //    sldPidThrottle.Value = Convert.ToDouble(elemList[i].InnerText);

            }
        }

        private void btnClear_Click2(object sender, RoutedEventArgs e)
        {
            ClearGraphs2();
        }

        /// <summary>
        /// This method resets all graphs
        /// </summary>
        void ClearGraphs2()
        {
            foreach (ZedGraphControl graph in graphs2)
            {
                if (graph.GraphPane.CurveList.Count > 0)
                {
                    foreach (CurveItem cl in graph.GraphPane.CurveList)
                    {
                        IPointListEdit ip = (IPointListEdit)cl.Points;
                        ip.Clear();
                    }
                }
            }
        }

        private void cbbPx4flow_DropDownClosed(object sender, EventArgs e)
        {
            ComboBox combobox = sender as ComboBox;

            if (combobox.SelectedIndex >= 0)
            {
                cbbPx4flow.SelectedIndex = combobox.SelectedIndex;
                cbbCam.SelectedIndex = combobox.SelectedIndex;

                byte[] robotPacket = new byte[32];
                robotPacket[0] = (byte)TJCommandID.EnableDisablePX4CmdID;
                robotPacket[2] = (byte)combobox.SelectedIndex;

                if (combobox.SelectedIndex == 1)
                    btnGetFrame.IsEnabled = true;
                else
                    btnGetFrame.IsEnabled = false;

                TJDragonfly.EnqueueTXPacket(robotPacket);
            }
        }
    }
}
