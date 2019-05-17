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
          //J1
          {
           .Kp          =   0.0005,
           .Ki          =   0.0001,
           .Kd          =   0.0001,
           .DcBias      =   0.27,
           .iState      =   0,
           .iMin        =  -2.0,
           .iMax        =   2.0,
           .dState      =   0,
           .Target      =   0,
           .TargetMin   =  -5000,
           .TargetMax   =   5000,
           .Output      =   0,
           .OutputMin   =  -0.75,
           .OutputMax   =   0.75,
           .Threshold   =   0.05
          },
          //J2
          {
           .Kp          =   0.0005,
           .Ki          =   0.0001,
           .Kd          =   0.0000,
           .DcBias      =   0.3000,
           .iState      =   0,
           .iMin        =  -2.0,
           .iMax        =   2.0,
           .dState      =   0,
           .Target      =   0,
           .TargetMin   =  -5000,
           .TargetMax   =   5000,
           .Output      =   0,
           .OutputMin   =  -0.8,
           .OutputMax   =   0.8,
           .Threshold   =   0.05
          },
          //J3
          {
           .Kp          = 0,
           .Ki          = 0,
           .Kd          = 0,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        = 0,
           .iMax        = 0,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   = 0,
           .TargetMax   = 0,
           .Output      = 0,
           .OutputMin   = 0,
           .OutputMax   = 0,
           .Threshold   = 0.01
          },
          //J4
          {
           .Kp          = 0,
           .Ki          = 0,
           .Kd          = 0,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        = 0,
           .iMax        = 0,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   = 0,
           .TargetMax   = 0,
           .Output      = 0,
           .OutputMin   = 0,
           .OutputMax   = 0,
           .Threshold   = 0.01
          },
          //J5
          {
           .Kp          = 0,
           .Ki          = 0,
           .Kd          = 0,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        = 0,
           .iMax        = 0,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   = 0,
           .TargetMax   = 0,
           .Output      = 0,
           .OutputMin   = 0,
           .OutputMax   = 0,
           .Threshold   = 0.01
          },
          //J6
          {
           .Kp          = 0,
           .Ki          = 0,
           .Kd          = 0,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        = 0,
           .iMax        = 0,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   = 0,
           .TargetMax   = 0,
           .Output      = 0,
           .OutputMin   = 0,
           .OutputMax   = 0,
           .Threshold   = 0.01
          }
};

static sPID PositionPIDs[JOINT_COUNT] = {
          {//Joint 1
           .Kp          = 100,
           .Ki          = 0,
           .Kd          = 0,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        =-100,
           .iMax        = 100,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   =-300,
           .TargetMax   = 300,
           .Output      = 0,
           .OutputMin   =-5000,
           .OutputMax   = 5000,
           .Threshold   = 0.00
          },

          {
           .Kp          = 100,
           .Ki          = 0,
           .Kd          = 0,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        =-100,
           .iMax        = 100,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   =-300,
           .TargetMax   = 300,
           .Output      = 0,
           .OutputMin   =-5000,
           .OutputMax   = 5000,
           .Threshold   = 0.00
          },
          {J3_A_KP,
           J3_A_KI,
           J3_A_KD,
           0,
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
