#include "ControlGlobals.h"
#include "Globals.h"

/*
*  Speed Control Defines
*/
#define     J1_S_KP     0.002
#define     J1_S_KI     0.0001
#define     J1_S_KD     0.00003
#define     J1_S_IMIN  -2
#define     J1_S_IMAX   2
#define     J1_S_TMIN  -2000
#define     J1_S_TMAX   2000
#define     J1_S_OMIN  -0.8
#define     J1_S_OMAX   0.8
#define     J2_S_KP     0.003
#define     J2_S_KI     0.00008
#define     J2_S_KD     0.0001
#define     J2_S_IMIN  -2
#define     J2_S_IMAX   2
#define     J2_S_TMIN  -2000
#define     J2_S_TMAX   2000
#define     J2_S_OMIN  -0.75
#define     J2_S_OMAX   0.75
#define     J3_S_KP     0.003
#define     J3_S_KI     0.00008
#define     J3_S_KD     0.0001
#define     J3_S_IMIN  -2
#define     J3_S_IMAX   2
#define     J3_S_TMIN  -2000
#define     J3_S_TMAX   2000
#define     J3_S_OMIN  -0.75
#define     J3_S_OMAX   0.75
#define     J4_S_KP     0
#define     J4_S_KI     0
#define     J4_S_KD     0
#define     J4_S_IMIN   0
#define     J4_S_IMAX   0
#define     J4_S_TMIN   0
#define     J4_S_TMAX   0
#define     J4_S_OMIN   0
#define     J4_S_OMAX   0
#define     J5_S_KP     0.0015
#define     J5_S_KI     0.0001
#define     J5_S_KD     0.00001
#define     J5_S_IMIN  -2
#define     J5_S_IMAX   2
#define     J5_S_TMIN  -2000
#define     J5_S_TMAX   2000
#define     J5_S_OMIN  -0.75
#define     J5_S_OMAX   0.75
#define     J6_S_KP     0
#define     J6_S_KI     0
#define     J6_S_KD     0
#define     J6_S_IMIN   0
#define     J6_S_IMAX   0
#define     J6_S_TMIN   0
#define     J6_S_TMAX   0
#define     J6_S_OMIN   0
#define     J6_S_OMAX   0
/*
*  Angle Control Defines
*/
#define     J1_A_KP     0.025
#define     J1_A_KI     0.00008
#define     J1_A_KD     0
#define     J1_A_IMIN  -0.8
#define     J1_A_IMAX   0.8
#define     J1_A_TMIN  -85
#define     J1_A_TMAX   115
#define     J1_A_OMIN  -0.8
#define     J1_A_OMAX   0.8
#define     J2_A_KP     0.045
#define     J2_A_KI     0.0003
#define     J2_A_KD     0
#define     J2_A_IMIN  -.7
#define     J2_A_IMAX   .7
#define     J2_A_TMIN  -65
#define     J2_A_TMAX   212
#define     J2_A_OMIN  -0.8
#define     J2_A_OMAX   0.8
#define     J3_A_KP     0.045
#define     J3_A_KI     0.0003
#define     J3_A_KD     0
#define     J3_A_IMIN  -.7
#define     J3_A_IMAX   .7
#define     J3_A_TMIN  -147.5
#define     J3_A_TMAX   147.5
#define     J3_A_OMIN  -0.8
#define     J3_A_OMAX   0.8
#define     J4_A_KP     0
#define     J4_A_KI     0
#define     J4_A_KD     0
#define     J4_A_IMIN   0
#define     J4_A_IMAX   0
#define     J4_A_TMIN   0
#define     J4_A_TMAX   0
#define     J4_A_OMIN   0
#define     J4_A_OMAX   0
#define     J5_A_KP     0.05
#define     J5_A_KI     0.00005
#define     J5_A_KD     0.01
#define     J5_A_IMIN  -0.6
#define     J5_A_IMAX   0.6
#define     J5_A_TMIN  -115
#define     J5_A_TMAX   115
#define     J5_A_OMIN  -0.6
#define     J5_A_OMAX   0.6
#define     J6_A_KP     0
#define     J6_A_KI     0
#define     J6_A_KD     0
#define     J6_A_IMIN   0
#define     J6_A_IMAX   0
#define     J6_A_TMIN   0
#define     J6_A_TMAX   0
#define     J6_A_OMIN   0
#define     J6_A_OMAX   0

static sPID SpeedPIDs[JOINT_COUNT] = {
          {J1_S_KP,
           J1_S_KI,
           J1_S_KD,
           0,
           J1_S_IMIN,
           J1_S_IMAX,
           0,
           0,
           J1_S_TMIN,
           J1_S_TMAX,
           0,
           J1_S_OMIN,
           J1_S_OMAX
          },

          {J2_S_KP,
           J2_S_KI,
           J2_S_KD,
           0,
           J2_S_IMIN,
           J2_S_IMAX,
           0,
           0,
           J2_S_TMIN,
           J2_S_TMAX,
           0,
           J2_S_OMIN,
           J2_S_OMAX
          },
          {J3_S_KP,
           J3_S_KI,
           J3_S_KD,
           0,
           J3_S_IMIN,
           J3_S_IMAX,
           0,
           0,
           J3_S_TMIN,
           J3_S_TMAX,
           0,
           J3_S_OMIN,
           J3_S_OMAX
          },
          {J4_S_KP,
           J4_S_KI,
           J4_S_KD,
           0,
           J4_S_IMIN,
           J4_S_IMAX,
           0,
           0,
           J4_S_TMIN,
           J4_S_TMAX,
           0,
           J4_S_OMIN,
           J4_S_OMAX
          },
          {J5_S_KP,
           J5_S_KI,
           J5_S_KD,
           0,
           J5_S_IMIN,
           J5_S_IMAX,
           0,
           0,
           J5_S_TMIN,
           J5_S_TMAX,
           0,
           J5_S_OMIN,
           J5_S_OMAX
          },
          {J6_S_KP,
           J6_S_KI,
           J6_S_KD,
           0,
           J6_S_IMIN,
           J6_S_IMAX,
           0,
           0,
           J6_S_TMIN,
           J6_S_TMAX,
           0,
           J6_S_OMIN,
           J6_S_OMAX
          },
};

static sPID PositionPIDs[JOINT_COUNT] = {
          {J1_A_KP,
           J1_A_KI,
           J1_A_KD,
           0,
           J1_A_IMIN,
           J1_A_IMAX,
           0,
           0,
           J1_A_TMIN,
           J1_A_TMAX,
           0,
           J1_A_OMIN,
           J1_A_OMAX
          },

          {J2_A_KP,
           J2_A_KI,
           J2_A_KD,
           0,
           J2_A_IMIN,
           J2_A_IMAX,
           0,
           0,
           J2_A_TMIN,
           J2_A_TMAX,
           0,
           J2_A_OMIN,
           J2_A_OMAX
          },
          {J3_A_KP,
           J3_A_KI,
           J3_A_KD,
           0,
           J3_A_IMIN,
           J3_A_IMAX,
           0,
           0,
           J3_A_TMIN,
           J3_A_TMAX,
           0,
           J3_A_OMIN,
           J3_A_OMAX
          },
          {J4_A_KP,
           J4_A_KI,
           J4_A_KD,
           0,
           J4_A_IMIN,
           J4_A_IMAX,
           0,
           0,
           J4_A_TMIN,
           J4_A_TMAX,
           0,
           J4_A_OMIN,
           J4_A_OMAX
          },
          {J5_A_KP,
           J5_A_KI,
           J5_A_KD,
           0,
           J5_A_IMIN,
           J5_A_IMAX,
           0,
           0,
           J5_A_TMIN,
           J5_A_TMAX,
           0,
           J5_A_OMIN,
           J5_A_OMAX
          },
          {J6_A_KP,
           J6_A_KI,
           J6_A_KD,
           0,
           J6_A_IMIN,
           J6_A_IMAX,
           0,
           0,
           J6_A_TMIN,
           J6_A_TMAX,
           0,
           J6_A_OMIN,
           J6_A_OMAX
          },
};

sPID* GetSpeedPIDs(void){
    return SpeedPIDs;
}

sPID* GetPositionPIDs(void){
    return PositionPIDs;
}
