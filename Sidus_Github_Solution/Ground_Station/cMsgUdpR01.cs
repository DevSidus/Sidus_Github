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
        public float pidRatePitchOutput { get; set; }
        public float pidRatePitchPresult { get; set; }
        public float pidRatePitchIresult { get; set; }
        public float pidRatePitchDresult { get; set; }

        public byte pidAnglePitchKp { get; set; }
        public byte pidAnglePitchKi { get; set; }
        public byte pidAnglePitchKd { get; set; }
        public float pidAnglePitchOutput { get; set; }
        public float pidAnglePitchPresult { get; set; }
        public float pidAnglePitchIresult { get; set; }
        public float pidAnglePitchDresult { get; set; }

        public byte pidRateYawKp { get; set; }
        public byte pidRateYawKi { get; set; }
        public byte pidRateYawKd { get; set; }
        public float pidRateYawOutput { get; set; }
        public float pidRateYawPresult { get; set; }
        public float pidRateYawIresult { get; set; }
        public float pidRateYawDresult { get; set; }

        public byte pidAngleYawKp { get; set; }
        public byte pidAngleYawKi { get; set; }
        public byte pidAngleYawKd { get; set; }
        public float pidAngleYawOutput { get; set; }
        public float pidAngleYawPresult { get; set; }
        public float pidAngleYawIresult { get; set; }
        public float pidAngleYawDresult { get; set; }
        public float commandedYawAngle { get; set; }

        public byte pidPosAltKp { get; set; }
        public byte pidPosAltKi { get; set; }
        public byte pidPosAltKd { get; set; }
        public float pidPosAltOutput { get; set; }
        public float pidPosAltPresult { get; set; }
        public float pidPosAltIresult { get; set; }
        public float pidPosAltDresult { get; set; }

        public byte pidVelAltKp { get; set; }
        public byte pidVelAltKi { get; set; }
        public byte pidVelAltKd { get; set; }
        public float pidVelAltOutput { get; set; }
        public float pidVelAltPresult { get; set; }
        public float pidVelAltIresult { get; set; }
        public float pidVelAltDresult { get; set; }

        public byte pidAccAltKp { get; set; }
        public byte pidAccAltKi { get; set; }
        public byte pidAccAltKd { get; set; }
        public float pidAccAltOutput { get; set; }
        public float pidAccAltPresult { get; set; }
        public float pidAccAltIresult { get; set; }
        public float pidAccAltDresult { get; set; }

        public byte pidPosXKp { get; set; }
        public byte pidPosXKi { get; set; }
        public byte pidPosXKd { get; set; }
        public float pidPosXOutput { get; set; }
        public float pidPosXPresult { get; set; }
        public float pidPosXIresult { get; set; }
        public float pidPosXDresult { get; set; }

        public byte pidVelXKp { get; set; }
        public byte pidVelXKi { get; set; }
        public byte pidVelXKd { get; set; }
        public float pidVelXOutput { get; set; }
        public float pidVelXPresult { get; set; }
        public float pidVelXIresult { get; set; }
        public float pidVelXDresult { get; set; }

        public byte pidAccXKp { get; set; }
        public byte pidAccXKi { get; set; }
        public byte pidAccXKd { get; set; }
        public float pidAccXOutput { get; set; }
        public float pidAccXPresult { get; set; }
        public float pidAccXIresult { get; set; }
        public float pidAccXDresult { get; set; }

        public byte gpsStatus { get; set; }
        public double gpsLat { get; set; }
        public double gpsLon { get; set; }
        public double gpsAlt { get; set; }
        public double homeLat { get; set; }
        public double homeLon { get; set; }
        public double homeAlt { get; set; }
        public double gpsVelN { get; set; }
        public double gpsVelE { get; set; }
        public double gpsVelD { get; set; }
        public double gpsPosAccuracy { get; set; }
        public double gpsVelAccuracy { get; set; }

        public byte posHoldAvailable { get; set; }
        public byte velHoldAvailable { get; set; }

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
