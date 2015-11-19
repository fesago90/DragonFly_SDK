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

namespace TechJectDF.TJCommands
{
    public class TJMoveCmd : TJCommand
    {
        int Throttle
        { get; set; }

        public TJMoveCmd(byte throttle)
            : base(TJCommandID.ThrottleCmdID)
        {
            Throttle = throttle;
        }
    }
}
