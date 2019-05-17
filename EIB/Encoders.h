#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>
#include <stdbool.h>
#include "Globals.h"

#define FILTER_WEIGHT   0.8
#define SAMPLE_RATE     800

typedef struct sEncoder_Tag{
    int32_t EncoderCount;
    float   Degrees;
    float   Speed;
    bool    NotMoving;

    float   MinEncoderCount;
    float   MaxEncoderCount;
    float   MinDegrees;
    float   MaxDegrees;
} sEncoder;

void Enc_Initialize(void);
void Enc_ResetEncoder(JOINT_POSITION Joint);
sEncoder* Enc_GetJointEncoder(JOINT_POSITION Joint);

#endif
