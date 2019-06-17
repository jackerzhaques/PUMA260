#include "ControlGlobals.h"
#include "Globals.h"

static sPID SpeedPIDs[JOINT_COUNT] = {
          //J1
          {
           .Kp          =   0.0001,
           .Ki          =   0.00001,
           .Kd          =   0.0000,
           .DcBias      =   0.1,
           .iState      =   0,
           .iMin        =  -1.0,
           .iMax        =   1.0,
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
           .Kp          =   0.00002,
           .Ki          =   0.0000011,
           .Kd          =   0.0000,
           .DcBias      =   0.3,
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
           .Kp          =   0.00002,
           .Ki          =   0.0000011,
           .Kd          =   0.0000,
           .DcBias      =   0.3,
           .iState      =   0,
           .iMin        =  -1.0,
           .iMax        =   1.0,
           .dState      =   0,
           .Target      =   0,
           .TargetMin   =  -5000,
           .TargetMax   =   5000,
           .Output      =   0,
           .OutputMin   =  -0.8,
           .OutputMax   =   0.8,
           .Threshold   =   0.05
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
           .Kp          =   0.00002,
           .Ki          =   0.000009,
           .Kd          =   0.0000,
           .DcBias      =   0.3,
           .iState      =   0,
           .iMin        =  -1.0,
           .iMax        =   1.0,
           .dState      =   0,
           .Target      =   0,
           .TargetMin   =  -5000,
           .TargetMax   =   5000,
           .Output      =   0,
           .OutputMin   =  -0.8,
           .OutputMax   =   0.8,
           .Threshold   =   0.05
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
           .Kp          = 10000,
           .Ki          = 0,
           .Kd          = 1000,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        =-100,
           .iMax        = 100,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   =-185,
           .TargetMax   = 105,
           .Output      = 0,
           .OutputMin   =-5000,
           .OutputMax   = 5000,
           .Threshold   = 0.00
          },
          {//Joint 2
           .Kp          = 10000,
           .Ki          = 0,
           .Kd          = 1000,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        =-100,
           .iMax        = 100,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   =-247.5,
           .TargetMax   = 67.5,
           .Output      = 0,
           .OutputMin   =-5000,
           .OutputMax   = 5000,
           .Threshold   = 0.00
          },
          {//Joint 3
           .Kp          = 10000,
           .Ki          = 0,
           .Kd          = 1000,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        =-100,
           .iMax        = 100,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   =-147.5,
           .TargetMax   = 147.5,
           .Output      = 0,
           .OutputMin   =-5000,
           .OutputMax   = 5000,
           .Threshold   = 0.00
          },
          {0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0
          },
          {//Joint 5
           .Kp          = 10000,
           .Ki          = 0,
           .Kd          = 1000,
           .DcBias      = 0,
           .iState      = 0,
           .iMin        =-100,
           .iMax        = 100,
           .dState      = 0,
           .Target      = 0,
           .TargetMin   =-115,
           .TargetMax   = 115,
           .Output      = 0,
           .OutputMin   =-5000,
           .OutputMax   = 5000,
           .Threshold   = 0.00
          },
          {0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0
          },
};

sPID* GetSpeedPIDs(void){
    return SpeedPIDs;
}

sPID* GetPositionPIDs(void){
    return PositionPIDs;
}
