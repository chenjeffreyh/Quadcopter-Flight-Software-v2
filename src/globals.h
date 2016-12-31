#ifndef _MPU6050_H_
  #include "MPU6050\MPU6050_6Axis_MotionApps20.h"
  #define _MPU6050_H_
#endif

#ifndef _PID_H_
  #include "pid\pid.h"
  #define _PID_H_
#endif

#define PID_YAW_P     0
#define PID_YAW_I     0
#define PID_YAW_D     0
#define PID_PITCH_P   0.02
#define PID_PITCH_I   0
#define PID_PITCH_D   0
#define PID_ROLL_P    PID_PITCH_P
#define PID_ROLL_I    PID_PITCH_I
#define PID_ROLL_D    PID_PITCH_D

#define MOTOR_OUTPUT_MAX_ABS 80
#define MOTOR_OUTPUT_MAX_USR 60

#define ARRAY_YAW 0
#define ARRAY_PITCH 1
#define ARRAY_ROLL 2

MPU6050 imu1;

PID pidYaw(PID_YAW_P, PID_YAW_I, PID_YAW_D);
PID pidPitch(PID_PITCH_P, PID_PITCH_I, PID_PITCH_D);
PID pidRoll(PID_ROLL_P, PID_ROLL_I, PID_ROLL_D);

int16_t output_fl, output_fr, output_rl, output_rr;
uint8_t baseThrottle;
int16_t targetYPR[3];
int16_t readYPR[3];
int16_t outputYPR[3];
