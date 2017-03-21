﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Ground_Station
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct structMsgUdpT01
    {
        public byte pidCommandState { get; set; }
        public byte pidRatePitchRollKp { get; set; }
        public byte pidRatePitchRollKi { get; set; }
        public byte pidRatePitchRollKd { get; set; }
        public byte pidRatePitchRollF1 { get; set; }
        public byte pidRatePitchRollF2 { get; set; }

        public byte pidAnglePitchRollKp { get; set; }
        public byte pidAnglePitchRollKi { get; set; }
        public byte pidAnglePitchRollKd { get; set; }
        public byte pidAnglePitchRollF1 { get; set; }
        public byte pidAnglePitchRollF2 { get; set; }

        public byte pidRateYawKp { get; set; }
        public byte pidRateYawKi { get; set; }
        public byte pidRateYawKd { get; set; }
        public byte pidRateYawF1 { get; set; }
        public byte pidRateYawF2 { get; set; }

        public byte pidAngleYawKp { get; set; }
        public byte pidAngleYawKi { get; set; }
        public byte pidAngleYawKd { get; set; }
        public byte pidAngleYawF1 { get; set; }
        public byte pidAngleYawF2 { get; set; }

        public byte pidAnglePitchRollOutFilter { get; set; }
        public byte pidAngleYawOutFilter { get; set; }

    }
    class cMsgUdpT01
    {
        public structMsgUdpT01 message;
        public byte[] dataBytes;

        public cMsgUdpT01()
        {
            dataBytes = new byte[Marshal.SizeOf(message)];            
        }
        public byte[] getPacket()
        {
            IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(message));
            Marshal.StructureToPtr(message, ptr, true);
            Marshal.Copy(ptr, dataBytes, 0, Marshal.SizeOf(message));
            Marshal.FreeHGlobal(ptr);
            return dataBytes;
        }
        public void setPacket()
        {
            IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(message));
            Marshal.Copy(dataBytes, 0, ptr, Marshal.SizeOf(message));
            message = (structMsgUdpT01)Marshal.PtrToStructure(ptr, message.GetType());
            Marshal.FreeHGlobal(ptr);            
        }
    };
}
