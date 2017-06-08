using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Ground_Station
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct structMsgUdpR01
    {
        public UInt32 timeStamp { get; set; }
        public byte statusMpu { get; set; }
        public byte statusBaro { get; set; }
        public byte statusCompass { get; set; }
        public byte statusUdp { get; set; }
        public byte statusGS { get; set; }

        //Gyro Measurements
        public short mpuGyroX { get; set; }
        public short mpuGyroY { get; set; }
        public short mpuGyroZ { get; set; }

        //Accelerometer Measurements with Gravity
        public short mpuAccX { get; set; }
        public short mpuAccY { get; set; }
        public short mpuAccZ { get; set; }

        //Accelerometer Measurements without Gravity
        public short mpuAccWorldX { get; set; }
        public short mpuAccWorldY { get; set; }
        public short mpuAccWorldZ { get; set; }

        //Yaw Pitch Roll Measurements in Radians
        public float mpuYaw { get; set; }
        public float mpuPitch { get; set; }
        public float mpuRoll { get; set; }

        public float baroTemp { get; set; }
        public float baroAlt { get; set; }

        public float compassHdg { get; set; }

        public float batteryVoltage { get; set; }

        public float quadVelocityWorldZ { get; set; }
        public float quadPositionWorldZ { get; set; }
        
        public byte modeQuad { get; set; }
        public byte autoModeStatus { get; set; }
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
        public byte pidAnglePitchOutFilter { get; set; }

        public byte pidRateRollKp { get; set; }
        public byte pidRateRollKi { get; set; }
        public byte pidRateRollKd { get; set; }
        public short pidRateRollOutput { get; set; }
        public short pidRateRollPresult { get; set; }
        public short pidRateRollIresult { get; set; }
        public short pidRateRollDresult { get; set; }
        public byte pidRateRollF1 { get; set; }
        public byte pidRateRollF2 { get; set; }

        public byte pidAngleRollKp { get; set; }
        public byte pidAngleRollKi { get; set; }
        public byte pidAngleRollKd { get; set; }
        public short pidAngleRollOutput { get; set; }
        public short pidAngleRollPresult { get; set; }
        public short pidAngleRollIresult { get; set; }
        public short pidAngleRollDresult { get; set; }
        public byte pidAngleRollF1 { get; set; }
        public byte pidAngleRollF2 { get; set; }
        public byte pidAngleRollOutFilter { get; set; }

        public byte pidRateYawKp { get; set; }
        public byte pidRateYawKi { get; set; }
        public byte pidRateYawKd { get; set; }
        public short pidRateYawOutput { get; set; }
        public short pidRateYawPresult { get; set; }
        public short pidRateYawIresult { get; set; }
        public short pidRateYawDresult { get; set; }
        public byte pidRateYawF1 { get; set; }
        public byte pidRateYawF2 { get; set; }

        public byte pidAngleYawKp { get; set; }
        public byte pidAngleYawKi { get; set; }
        public byte pidAngleYawKd { get; set; }
        public short pidAngleYawOutput { get; set; }
        public short pidAngleYawPresult { get; set; }
        public short pidAngleYawIresult { get; set; }
        public short pidAngleYawDresult { get; set; }
        public byte pidAngleYawF1 { get; set; }
        public byte pidAngleYawF2 { get; set; }
        public byte pidAngleYawOutFilter { get; set; }

        public short commandedYawAngle { get; set; }

        public byte pidVelAltKp { get; set; }
        public byte pidVelAltKi { get; set; }
        public byte pidVelAltKd { get; set; }
        public short pidVelAltOutput { get; set; }
        public short pidVelAltPresult { get; set; }
        public short pidVelAltIresult { get; set; }
        public short pidVelAltDresult { get; set; }
        public byte pidVelAltF1 { get; set; }
        public byte pidVelAltF2 { get; set; }

        public byte pidAccAltKp { get; set; }
        public byte pidAccAltKi { get; set; }
        public byte pidAccAltKd { get; set; }
        public short pidAccAltOutput { get; set; }
        public short pidAccAltPresult { get; set; }
        public short pidAccAltIresult { get; set; }
        public short pidAccAltDresult { get; set; }
        public byte pidAccAltF1 { get; set; }
        public byte pidAccAltF2 { get; set; }
        public byte pidAccAltOutFilter { get; set; }
    }
    class cMsgUdpR01
    {
        public structMsgUdpR01 message;
        public byte[] dataBytes;

        public cMsgUdpR01()
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
            message = (structMsgUdpR01)Marshal.PtrToStructure(ptr, message.GetType());
            Marshal.FreeHGlobal(ptr);
        }
    };
}
