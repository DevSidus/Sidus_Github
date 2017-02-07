using System;
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
        public byte pidRatePitchKp { get; set; }
        public byte pidRatePitchKi { get; set; }
        public byte pidRatePitchKd { get; set; }
        public byte pidRatePitchF1 { get; set; }
        public byte pidRatePitchF2 { get; set; }

        public byte pidAnglePitchKp { get; set; }
        public byte pidAnglePitchKi { get; set; }
        public byte pidAnglePitchKd { get; set; }
        public byte pidAnglePitchF1 { get; set; }
        public byte pidAnglePitchF2 { get; set; }

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
