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
        public float mpuAccWorldX { get; set; }
        public float mpuAccWorldY { get; set; }
        public float mpuAccWorldZ { get; set; }

        //Yaw Pitch Roll Measurements in Radians
        public float mpuYaw { get; set; }
        public float mpuPitch { get; set; }
        public float mpuRoll { get; set; }

        public float baroTemp { get; set; }
        public float baroAlt { get; set; }

        public float compassHdg { get; set; }

        public float batteryVoltage { get; set; }

        public float ultrasonicDist { get; set; }

        // Position Parameter Estimation with Kalman Filter
        public float quadAccelerationWorldX { get; set; }
        public float quadVelocityWorldX { get; set; }
        public float quadPositionWorldX { get; set; }
        public float quadAccelerationWorldY { get; set; }
        public float quadVelocityWorldY { get; set; }
        public float quadPositionWorldY { get; set; }

        // Altitude Parameter Estimation with Kalman Filter
        public float quadAccelerationWorldZ { get; set; }
        public float quadVelocityWorldZ { get; set; }
        public float quadPositionWorldZ { get; set; }
        
        public byte modeQuad { get; set; }
        public byte autoModeStatus { get; set; }
        public byte statusRx { get; set; }
        public short rxThrottle { get; set; }
        public short rxPitch { get; set; }
        public short rxRoll { get; set; }
        public short rxYaw { get; set; }
        public short rx6thCh { get; set; }

        public byte pidRatePitchKp { get; set; }
        public byte pidRatePitchKi { get; set; }
        public byte pidRatePitchKd { get; set; }
        public short pidRatePitchOutput { get; set; }
        public short pidRatePitchPresult { get; set; }
        public short pidRatePitchIresult { get; set; }
        public short pidRatePitchDresult { get; set; }

        public byte pidAnglePitchKp { get; set; }
        public byte pidAnglePitchKi { get; set; }
        public byte pidAnglePitchKd { get; set; }
        public short pidAnglePitchOutput { get; set; }
        public short pidAnglePitchPresult { get; set; }
        public short pidAnglePitchIresult { get; set; }
        public short pidAnglePitchDresult { get; set; }

        //public byte pidRateRollKp { get; set; }
        //public byte pidRateRollKi { get; set; }
        //public byte pidRateRollKd { get; set; }
        //public short pidRateRollOutput { get; set; }
        //public short pidRateRollPresult { get; set; }
        //public short pidRateRollIresult { get; set; }
        //public short pidRateRollDresult { get; set; }

        //public byte pidAngleRollKp { get; set; }
        //public byte pidAngleRollKi { get; set; }
        //public byte pidAngleRollKd { get; set; }
        //public short pidAngleRollOutput { get; set; }
        //public short pidAngleRollPresult { get; set; }
        //public short pidAngleRollIresult { get; set; }
        //public short pidAngleRollDresult { get; set; }

        public byte pidRateYawKp { get; set; }
        public byte pidRateYawKi { get; set; }
        public byte pidRateYawKd { get; set; }
        public short pidRateYawOutput { get; set; }
        public short pidRateYawPresult { get; set; }
        public short pidRateYawIresult { get; set; }
        public short pidRateYawDresult { get; set; }

        public byte pidAngleYawKp { get; set; }
        public byte pidAngleYawKi { get; set; }
        public byte pidAngleYawKd { get; set; }
        public short pidAngleYawOutput { get; set; }
        public short pidAngleYawPresult { get; set; }
        public short pidAngleYawIresult { get; set; }
        public short pidAngleYawDresult { get; set; }

        public short commandedYawAngle { get; set; }

        public byte pidPosAltKp { get; set; }
        public byte pidPosAltKi { get; set; }
        public byte pidPosAltKd { get; set; }
        public short pidPosAltOutput { get; set; }
        public short pidPosAltPresult { get; set; }
        public short pidPosAltIresult { get; set; }
        public short pidPosAltDresult { get; set; }

        public byte pidVelAltKp { get; set; }
        public byte pidVelAltKi { get; set; }
        public byte pidVelAltKd { get; set; }
        public short pidVelAltOutput { get; set; }
        public short pidVelAltPresult { get; set; }
        public short pidVelAltIresult { get; set; }
        public short pidVelAltDresult { get; set; }

        public byte pidAccAltKp { get; set; }
        public byte pidAccAltKi { get; set; }
        public byte pidAccAltKd { get; set; }
        public short pidAccAltOutput { get; set; }
        public short pidAccAltPresult { get; set; }
        public short pidAccAltIresult { get; set; }
        public short pidAccAltDresult { get; set; }

        public byte pidAccPosXKp { get; set; }
        public byte pidAccPosXKi { get; set; }
        public byte pidAccPosXKd { get; set; }
        public short pidAccPosXOutput { get; set; }
        public short pidAccPosXPresult { get; set; }
        public short pidAccPosXIresult { get; set; }
        public short pidAccPosXDresult { get; set; }

        public byte pidAccPosYKp { get; set; }
        public byte pidAccPosYKi { get; set; }
        public byte pidAccPosYKd { get; set; }
        public short pidAccPosYOutput { get; set; }
        public short pidAccPosYPresult { get; set; }
        public short pidAccPosYIresult { get; set; }
        public short pidAccPosYDresult { get; set; }

        public byte gpsStatus { get; set; }
        public double gpsLat { get; set; }
        public double gpsLon { get; set; }
        public double gpsAlt { get; set; }
        public double homeLat { get; set; }
        public double homeLon { get; set; }
        public double homeAlt { get; set; }
        public double gpsVelN { get; set; }
        public double gpsVelE { get; set; }
        public double gpsPosAccuracy { get; set; }
        public double gpsVelAccuracy { get; set; }


        public ushort lidar_distance { get; set; }
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
