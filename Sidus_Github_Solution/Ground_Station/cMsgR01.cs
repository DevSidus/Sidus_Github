using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Ground_Station
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct structMsgR01
    {
        public char startChar1 { get; set; }
        public char startChar2 { get; set; }
        public byte modeQuad { get; set; }
        public byte statusRx { get; set; }
        public short rxThrottle { get; set; }
        public short rxPitch { get; set; }
        public short rxRoll { get; set; }
        public short rxYaw { get; set; }

        public byte pidRatePitchKp { get; set; }
        public byte pidRatePitchKi { get; set; }
        public byte pidRatePitchKd { get; set; }
        public short pidRatePitchOutput { get; set; }
        public short pidRatePitchPresult { get; set; }
        public short pidRatePitchIresult { get; set; }
        public short pidRatePitchDresult { get; set; }
        public byte pidRatePitchF1 { get; set; }
        public byte pidRatePitchF2 { get; set; }

        public byte pidAnglePitchKp { get; set; }
        public byte pidAnglePitchKi { get; set; }
        public byte pidAnglePitchKd { get; set; }
        public short pidAnglePitchOutput { get; set; }
        public short pidAnglePitchPresult { get; set; }
        public short pidAnglePitchIresult { get; set; }
        public short pidAnglePitchDresult { get; set; }
        public byte pidAnglePitchF1 { get; set; }
        public byte pidAnglePitchF2 { get; set; }
        public byte pidRateRollKp { get; set; }
        public byte pidRateRollKi { get; set; }
        public byte pidRateRollKd { get; set; }
        public short pidRateRollOutput { get; set; }
        public short pidRateRollPresult { get; set; }
        public short pidRateRollIresult { get; set; }
        public short pidRateRollDresult { get; set; }
        public byte pidRateRollF1 { get; set; }
        public byte pidRateRollF2 { get; set; }

        public char endChar { get; set; }
    }
    class cMsgR01
    {
        public structMsgR01 message;
        public byte[] dataBytes;

        public cMsgR01()
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
            message = (structMsgR01)Marshal.PtrToStructure(ptr, message.GetType());
            Marshal.FreeHGlobal(ptr);
        }
    };
}
