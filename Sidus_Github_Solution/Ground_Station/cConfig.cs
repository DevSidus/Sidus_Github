using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Ground_Station
{
    static class cConfig
    {
        public static int UDP_PORT_NUM = 8080;
        public static string DEFAULT_REMOTE_IP = "192.168.1.10";

        public static double RESOLUTION_PID_KP = 0.01;
        public static double RESOLUTION_PID_KI = 0.001;
        public static double RESOLUTION_PID_KD = 0.001;
        public static double RESOLUTION_PID_F = 0.01;
    }

    enum pidCommandType {   pidCommandNoAction = 0,
                            pidCommandApplyRatePitchRoll = 1,
                            pidCommandApplyAnglePitchRoll = 2,
                            pidCommandApplyRateYaw = 3,
                            pidCommandApplyAngleYaw = 4,
                            pidCommandApplyAll = 5,
                            pidCommandApplyVelAlt = 6,
                            pidCommandApplyAccAlt = 7,
                            pidCommandApplyPosAlt = 8,
                            pidCommandApplyAccPos = 9,
    }


    enum autoModeType
    {
        autoModeOFF = 0,
        autoModeAltitude = 1,
    }

}
