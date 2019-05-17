#ifndef CONTROL_GLOBALS_H
#define CONTROL_GLOBALS_H

#define EMPTY_PID           {0,0,0,0,0,0,0,0,0,0,0,0,0};

typedef struct sPID_tag{
    float Kp;
    float Ki;
    float Kd;

    float DcBias;

    float iState;
    float iMin;
    float iMax;

    float dState;

    float Target;
    float TargetMin;
    float TargetMax;

    float Output;
    float OutputMin;
    float OutputMax;

    float Threshold;
} sPID;

typedef struct PositionVector_tag{
    float x;
    float y;
    float z;
    float theta;
} PositionVector;


sPID* GetSpeedPIDs(void);
sPID* GetPositionPIDs(void);

#endif
