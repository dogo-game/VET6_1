#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"

void positionServo(float ref, DJI_t * motor);

void speedServo(float ref, DJI_t * motor);
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);
#endif
