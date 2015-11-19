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
using System.Windows.Controls;

namespace CustomExtensions
{
    public static class TJCustomExtensions
    {
        public static void InvokeIfRequired(this Control c, Action action)
        {
            if (System.Threading.Thread.CurrentThread != c.Dispatcher.Thread)
            {
                c.Dispatcher.Invoke(action);
            }
            else
            {
                action();
            }
        }
    }
}
