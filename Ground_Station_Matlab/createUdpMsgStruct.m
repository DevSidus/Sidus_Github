function udpMsg_struct = createUdpMsgStruct()
% This structure is created from 'cMsgUdpR01.h'

udpMsg_struct = [ ...
	"uint32", "timeStamp";

	"uint8", "statusMpu";
	"uint8", "statusBaro";
	"uint8", "statusCompass";
	"uint8", "statusUdp";
	"uint8", "statusGS";

	% Gyro Measurements
	"int16", "mpuGyroX";
	"int16", "mpuGyroY";
	"int16", "mpuGyroZ";

	% Accelerometer Measurements with Gravity
	"int16", "mpuAccX";
	"int16", "mpuAccY";
	"int16", "mpuAccZ";

	% Accelerometer Measurements without Gravity
	"single", "mpuAccWorldX";
	"single", "mpuAccWorldY";
	"single", "mpuAccWorldZ";

	% Yaw Pitch Roll Measurements in Radians
	"single", "mpuYaw";
	"single", "mpuPitch";
	"single", "mpuRoll";

	"single", "baroTemp";
	"single", "baroAlt";

	"single", "compassHdg";

	"single", "batteryVoltage";

	"single", "ultrasonicDist";

	% Position Parameter Estimation with Kalman Filter
	"single", "quadAccelerationWorldX"; % kalman filtered output
	"single", "quadVelocityWorldX";   % kalman filtered output
	"single", "quadPositionWorldX";   % kalman filtered output
	"single", "quadAccelerationWorldY"; % kalman filtered output
	"single", "quadVelocityWorldY";   % kalman filtered output
	"single", "quadPositionWorldY";   % kalman filtered output

	% Altitude Parameter Estimation with Kalman Filter
	"single", "quadAccelerationWorldZ"; % kalman filtered output
	"single", "quadVelocityWorldZ";   % kalman filtered output
	"single", "quadPositionWorldZ";   % kalman filtered output
	
	"uint8", "modeQuad";
	"uint8", "autoModeStatus";
	"uint8", "statusRx";
	"int16", "rxThrottle";
	"int16", "rxPitch";
	"int16", "rxRoll";
	"int16", "rxYaw";
	"int16", "rx6thCh";

	"uint8", "pidRatePitchKp";
	"uint8", "pidRatePitchKi";
	"uint8", "pidRatePitchKd";
	"single", "pidRatePitchOutput";
	"single", "pidRatePitchPresult";
	"single", "pidRatePitchIresult";
	"single", "pidRatePitchDresult";

	"uint8", "pidAnglePitchKp";
	"uint8", "pidAnglePitchKi";
	"uint8", "pidAnglePitchKd";
	"single", "pidAnglePitchOutput";
	"single", "pidAnglePitchPresult";
	"single", "pidAnglePitchIresult";
	"single", "pidAnglePitchDresult";

	"uint8", "pidRateYawKp";
	"uint8", "pidRateYawKi";
	"uint8", "pidRateYawKd";
	"single", "pidRateYawOutput";
	"single", "pidRateYawPresult";
	"single", "pidRateYawIresult";
	"single", "pidRateYawDresult";

	"uint8", "pidAngleYawKp";
	"uint8", "pidAngleYawKi";
	"uint8", "pidAngleYawKd";
	"single", "pidAngleYawOutput";
	"single", "pidAngleYawPresult";
	"single", "pidAngleYawIresult";
	"single", "pidAngleYawDresult";
	"single", "commandedYawAngle";

	"uint8", "pidPosAltKp";
	"uint8", "pidPosAltKi";
	"uint8", "pidPosAltKd";
	"single", "pidPosAltOutput";
	"single", "pidPosAltPresult";
	"single", "pidPosAltIresult";
	"single", "pidPosAltDresult";

	"uint8", "pidVelAltKp";
	"uint8", "pidVelAltKi";
	"uint8", "pidVelAltKd";
	"single", "pidVelAltOutput";
	"single", "pidVelAltPresult";
	"single", "pidVelAltIresult";
	"single", "pidVelAltDresult";

	"uint8", "pidAccAltKp";
	"uint8", "pidAccAltKi";
	"uint8", "pidAccAltKd";
	"single", "pidAccAltOutput";
	"single", "pidAccAltPresult";
	"single", "pidAccAltIresult";
	"single", "pidAccAltDresult";

	"uint8", "pidPosXKp";
	"uint8", "pidPosXKi";
	"uint8", "pidPosXKd";
	"single", "pidPosXOutput";
	"single", "pidPosXPresult";
	"single", "pidPosXIresult";
	"single", "pidPosXDresult";

	"uint8", "pidVelXKp";
	"uint8", "pidVelXKi";
	"uint8", "pidVelXKd";
	"single", "pidVelXOutput";
	"single", "pidVelXPresult";
	"single", "pidVelXIresult";
	"single", "pidVelXDresult";

	"uint8", "pidAccXKp";
	"uint8", "pidAccXKi";
	"uint8", "pidAccXKd";
	"single", "pidAccXOutput";
	"single", "pidAccXPresult";
	"single", "pidAccXIresult";
	"single", "pidAccXDresult";

	% GPS DATA
	"uint8", "gpsStatus";
	"double", "gpsLat";
	"double", "gpsLon";
	"double", "gpsAlt";
	"double", "homeLat";
	"double", "homeLon";
	"double", "homeAlt";
	"double", "gpsVelN";
	"double", "gpsVelE";
	"double", "gpsVelD";
	"double", "gpsPosAccuracy";
	"double", "gpsVelAccuracy";
	
	"uint8", "posHoldAvailable";
	"uint8", "velHoldAvailable";

	% Lidar Distance
	"uint16", "lidar_distance";
];