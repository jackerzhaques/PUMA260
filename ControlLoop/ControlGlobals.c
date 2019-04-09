#include "ControlGlobals.h"
#include "Globals.h"

/*
*  Speed Control Defines
*/
#define     J1_S_KP     0.00001
#define     J1_S_KI     0.0000075
#define     J1_S_KD     0.000000
#define     J1_S_IMIN  -2
#define     J1_S_IMAX   2
#define     J1_S_TMIN  -2000
#define     J1_S_TMAX   2000
#define     J1_S_OMIN  -0.75
#define     J1_S_OMAX   0.75
#define     J2_S_KP     0.00008
#define     J2_S_KI     0.000014
#define     J2_S_KD     0.0000003
#define     J2_S_IMIN  -2
#define     J2_S_IMAX   2
#define     J2_S_TMIN  -2000
#define     J2_S_TMAX   2000
#define     J2_S_OMIN  -0.85
#define     J2_S_OMAX   0.85
#define     J3_S_KP     0
#define     J3_S_KI     0
#define     J3_S_KD     0
#define     J3_S_IMIN   0
#define     J3_S_IMAX   0
#define     J3_S_TMIN   0
#define     J3_S_TMAX   0
#define     J3_S_OMIN   0
#define     J3_S_OMAX   0
#define     J4_S_KP     0
#define     J4_S_KI     0
#define     J4_S_KD     0
#define     J4_S_IMIN   0
#define     J4_S_IMAX   0
#define     J4_S_TMIN   0
#define     J4_S_TMAX   0
#define     J4_S_OMIN   0
#define     J4_S_OMAX   0
#define     J5_S_KP     0
#define     J5_S_KI     0
#define     J5_S_KD     0
#define     J5_S_IMIN   0
#define     J5_S_IMAX   0
#define     J5_S_TMIN   0
#define     J5_S_TMAX   0
#define     J5_S_OMIN   0
#define     J5_S_OMAX   0
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
#define     J1_A_KP     0
#define     J1_A_KI     0
#define     J1_A_KD     0
#define     J1_A_IMIN   0
#define     J1_A_IMAX   0
#define     J1_A_TMIN   0
#define     J1_A_TMAX   0
#define     J1_A_OMIN   0
#define     J1_A_OMAX   0
#define     J2_A_KP     0
#define     J2_A_KI     0
#define     J2_A_KD     0
#define     J2_A_IMIN   0
#define     J2_A_IMAX   0
#define     J2_A_TMIN   0
#define     J2_A_TMAX   0
#define     J2_A_OMIN   0
#define     J2_A_OMAX   0
#define     J3_A_KP     0
#define     J3_A_KI     0
#define     J3_A_KD     0
#define     J3_A_IMIN   0
#define     J3_A_IMAX   0
#define     J3_A_TMIN   0
#define     J3_A_TMAX   0
#define     J3_A_OMIN   0
#define     J3_A_OMAX   0
#define     J4_A_KP     0
#define     J4_A_KI     0
#define     J4_A_KD     0
#define     J4_A_IMIN   0
#define     J4_A_IMAX   0
#define     J4_A_TMIN   0
#define     J4_A_TMAX   0
#define     J4_A_OMIN   0
#define     J4_A_OMAX   0
#define     J5_A_KP     0
#define     J5_A_KI     0
#define     J5_A_KD     0
#define     J5_A_IMIN   0
#define     J5_A_IMAX   0
#define     J5_A_TMIN   0
#define     J5_A_TMAX   0
#define     J5_A_OMIN   0
#define     J5_A_OMAX   0
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

sPID* GetSpeedPIDs(void){
    return SpeedPIDs;
}
