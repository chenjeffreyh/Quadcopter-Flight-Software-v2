#ifndef _MPU6050_H_
  #include "MPU6050\MPU6050_6Axis_MotionApps20.h"
  #define _MPU6050_H_
#endif

#ifndef _CONTROLLER_h_
  #include "Controller\Controller.h"
  #define _CONTROLLER_h_
#endif

#ifndef _PID_H_
  #include "pid\pid.h"
  #define _PID_H_
#endif

#define LED_STATUS_RED 4
#define LED_STATUS_YELLOW 7
#define LED_STATUS_GREEN 8

#define PID_YAW_P     0
#define PID_YAW_I     0
#define PID_YAW_D     0
#define PID_PITCH_P   0.5
#define PID_PITCH_I   0
#define PID_PITCH_D   0
#define PID_ROLL_P    PID_PITCH_P
#define PID_ROLL_I    PID_PITCH_I
#define PID_ROLL_D    PID_PITCH_D

#define MOTOR_OUTPUT_MAX_ABS 255.0
#define MOTOR_OUTPUT_MAX_USR 100

#define ARRAY_YAW 0
#define ARRAY_PITCH 1
#define ARRAY_ROLL 2

#define MPU_INTERRUPT 2

RF24 radio_base(A1, A0);
uint8_t radio_channel = 30;
Controller controller(&radio_base, radio_channel, true);
rx_values_t receiverVals;

MPU6050 imu1;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

PID pidYaw(PID_YAW_P, PID_YAW_I, PID_YAW_D);
PID pidPitch(PID_PITCH_P, PID_PITCH_I, PID_PITCH_D);
PID pidRoll(PID_ROLL_P, PID_ROLL_I, PID_ROLL_D);

int16_t output_fl, output_fr, output_rl, output_rr;
uint8_t baseThrottle;
double targetYPR[3];
double readYPR[3];
double outputYPR[3];
