using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using TechJectDF;
using TechJectDF.TJCommands;

namespace AlphaUtilityWF
{
    /// <summary>
    /// Interaction logic for Bootloader.xaml
    /// </summary>
    public partial class Bootloader : Window
    {
        //Stopwatch stopwatch;
        bool ConnectedToBootloader = false;
        bool FileSelected = false;
        string HexFileName = null;
        bool MCUResponded = false;
        bool MCURequestNext = false;
        bool MCURequestLast = false;

        byte pipe = 1;

        public Bootloader(byte p)
        {
            InitializeComponent();
            pipe = p;
            TJDragonfly.nRFChID = p;
            ConnectToDragonfly();
        }

        /// <summary>
        /// Setups up graphs and other UI components
        /// </summary>
        private void BootloaderWindow_Loaded(object sender, RoutedEventArgs e)
        {
            //stopwatch = Stopwatch.StartNew();
            //ConnectToDragonfly();
        }

        private void BootloaderWindow_Closed(object sender, EventArgs e)
        {
            TJDragonfly.DragonflyConnectionChanged -= TJDragonfly_DragonflyConnectionChanged;
            TJDragonfly.DragonflyBootloaderUpdate -= CheckBootloaderStatus;
            TJDragonfly.Disconnect();
        }

        private void btnSelectHex_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.Forms.OpenFileDialog openHexFileDialog = new System.Windows.Forms.OpenFileDialog();
            openHexFileDialog.Filter = "Hex Files (.hex)|*.hex";
            openHexFileDialog.FilterIndex = 1;

            openHexFileDialog.Multiselect = false;

            openHexFileDialog.ShowDialog();

            if (openHexFileDialog.FileName != "")
            {
                HexFileName = openHexFileDialog.FileName;
                if (ConnectedToBootloader == true)
                {
                    string s = "Upload: " + System.IO.Path.GetFileNameWithoutExtension(HexFileName);
                    Console.WriteLine(s);
                }
                else
                {
                    Console.WriteLine("You must select to connect to the bootloader");
                }

                btnUploadHex.IsEnabled = false;
                FileSelected = true;
            }
        }

        private void btnConnectBootloader_Click(object sender, RoutedEventArgs e)
        {
            if (ConnectedToBootloader)
            {
                ConnectedToBootloader = false;
                //Console.WriteLine("Don't Connect to Bootloader on Reset");
                if (FileSelected)
                {
                    Console.WriteLine("You must select a file to connect to the bootloader");
                    btnUploadHex.IsEnabled = false;
                }

            }
            else
            {
                TJConnectToBootloaderCmd cmd = new TJConnectToBootloaderCmd();
                prbConnectBootloader.Value = 50;
                TJDragonfly.EnqueueCommand(cmd);

                if (FileSelected)
                {
                    Console.WriteLine("Upload: " + System.IO.Path.GetFileNameWithoutExtension(HexFileName));
                    btnUploadHex.IsEnabled = true;
                }
                while (MCUResponded == false) ;
                MCUResponded = false;

                prbConnectBootloader.Value = 100;
            }
        }

        private void btnUploadHex_Click(object sender, RoutedEventArgs e)
        {
            System.IO.Stream fileStream = System.IO.File.Open(HexFileName, System.IO.FileMode.Open, System.IO.FileAccess.Read, System.IO.FileShare.Read);
            System.IO.FileInfo HexFileInfo = new System.IO.FileInfo(HexFileName);
            long BytesRead = 0;
            long TotalBytes = HexFileInfo.Length;
            int resending = 0;
            bool good = true;

            Console.WriteLine();
            using (System.IO.StreamReader reader = new System.IO.StreamReader(fileStream))
            {

                string line = reader.ReadLine();
                while (line != null)
                {
                    TJBootloaderDataCmd cmd = new TJBootloaderDataCmd(line);    // Create new command from line
                    TJDragonfly.EnqueueCommand(cmd);                            // Send that command
                    while (MCUResponded == false)                               // Wait for any response
                    {
                        if (TJDragonfly.getRFController() == 1)
                            System.Threading.Thread.Sleep(10);
                    }
                    prbConnectBootloader.Value = 100;                 // Any response indicates the bootloader is connected
                    MCUResponded = false;                                       // Reset response indicator

                    if (MCURequestNext == true)                                 // If it went through, request next line
                    {
                        resending = 0;
                        BytesRead += line.Length;                               // Update progress bar
                        prbUpload.Value = Convert.ToInt32((BytesRead * 100) / TotalBytes);

                        Console.WriteLine("Byte " + BytesRead + " of " + TotalBytes + ": " + line);
                        //Console.Write("Sent " + line );                                    // Write that line to console
                        Console.WriteLine();
                        line = reader.ReadLine();                               // Increment the line in the file

                        MCURequestNext = false;                                 // Reset indicator for the next packet
                    }
                    else
                    {
                        resending++;
                        Console.Write("Resending..." + line);
                        Console.WriteLine();

                        if (resending > 150)
                        {
                            MessageBox.Show(this, "Too many retries", "Error");
                            good = false;
                            TJDragonfly.DragonflyConnectionChanged -= TJDragonfly_DragonflyConnectionChanged;
                            TJDragonfly.DragonflyBootloaderUpdate -= CheckBootloaderStatus;
                            TJDragonfly.Disconnect();
                            break;
                            //this.Dispose();
                        }

                    }
                }

                if (good == true)
                    prbUpload.Value = 100;

                ConnectedToBootloader = false;

            }
            fileStream.Close();
        }

        void ConnectToDragonfly()
        {
            TJDragonfly.DragonflyConnectionChanged += TJDragonfly_DragonflyConnectionChanged;
            //TJDragonfly.Connect();
            if (TJDragonfly.Connect() == true)
            {
                Console.WriteLine("Connected...");
            }
        }

        /// <summary>
        /// This method responds to any changes in the status of the connection to the dragonfly.
        /// </summary>
        void TJDragonfly_DragonflyConnectionChanged()
        {
            if (TJDragonfly.IsConnected)
            {
                TJDragonfly.DragonflyBootloaderUpdate += CheckBootloaderStatus;
                TJCommand cmd = new TJCommand(TJCommandID.SetTogglesCmdID);
                TJDragonfly.EnqueueCommand(cmd);
            }
            else
            {
                TJDragonfly.DragonflyConnectionChanged -= TJDragonfly_DragonflyConnectionChanged;
                TJDragonfly.DragonflyBootloaderUpdate -= CheckBootloaderStatus;
            }
        }

        private void CheckBootloaderStatus(TJBootloaderResponsePacket obj)
        {
            //Console.WriteLine("MCU Response");
            MCUResponded = true;
            // Handle send last and send next
            if (obj.SendNextLine == 1) 
                MCURequestNext = true;
        }

        
    }
}
