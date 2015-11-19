﻿#pragma checksum "..\..\Bootloader.xaml" "{406ea660-64cf-4c82-b6f0-42d48172a799}" "328BF92F6A03D8FC9FE556812E7B2817"
//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.34014
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Forms.Integration;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;


namespace AlphaUtilityWF {
    
    
    /// <summary>
    /// Bootloader
    /// </summary>
    public partial class Bootloader : System.Windows.Window, System.Windows.Markup.IComponentConnector {
        
        
        #line 1 "..\..\Bootloader.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal AlphaUtilityWF.Bootloader BootloaderWindow;
        
        #line default
        #line hidden
        
        
        #line 6 "..\..\Bootloader.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Button btnSelectHex;
        
        #line default
        #line hidden
        
        
        #line 7 "..\..\Bootloader.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Button btnConnectBootloader;
        
        #line default
        #line hidden
        
        
        #line 8 "..\..\Bootloader.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Button btnUploadHex;
        
        #line default
        #line hidden
        
        
        #line 10 "..\..\Bootloader.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ProgressBar prbConnectBootloader;
        
        #line default
        #line hidden
        
        
        #line 12 "..\..\Bootloader.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ProgressBar prbUpload;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/AlphaUtilityWF;component/bootloader.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\Bootloader.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            this.BootloaderWindow = ((AlphaUtilityWF.Bootloader)(target));
            
            #line 4 "..\..\Bootloader.xaml"
            this.BootloaderWindow.Closed += new System.EventHandler(this.BootloaderWindow_Closed);
            
            #line default
            #line hidden
            
            #line 4 "..\..\Bootloader.xaml"
            this.BootloaderWindow.Loaded += new System.Windows.RoutedEventHandler(this.BootloaderWindow_Loaded);
            
            #line default
            #line hidden
            return;
            case 2:
            this.btnSelectHex = ((System.Windows.Controls.Button)(target));
            
            #line 6 "..\..\Bootloader.xaml"
            this.btnSelectHex.Click += new System.Windows.RoutedEventHandler(this.btnSelectHex_Click);
            
            #line default
            #line hidden
            return;
            case 3:
            this.btnConnectBootloader = ((System.Windows.Controls.Button)(target));
            
            #line 7 "..\..\Bootloader.xaml"
            this.btnConnectBootloader.Click += new System.Windows.RoutedEventHandler(this.btnConnectBootloader_Click);
            
            #line default
            #line hidden
            return;
            case 4:
            this.btnUploadHex = ((System.Windows.Controls.Button)(target));
            
            #line 8 "..\..\Bootloader.xaml"
            this.btnUploadHex.Click += new System.Windows.RoutedEventHandler(this.btnUploadHex_Click);
            
            #line default
            #line hidden
            return;
            case 5:
            this.prbConnectBootloader = ((System.Windows.Controls.ProgressBar)(target));
            return;
            case 6:
            this.prbUpload = ((System.Windows.Controls.ProgressBar)(target));
            return;
            }
            this._contentLoaded = true;
        }
    }
}

