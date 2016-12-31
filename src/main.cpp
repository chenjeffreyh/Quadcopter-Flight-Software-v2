#include <Arduino.h>
#include <Wire.h>
#include "globals.h"

#ifndef _MPU6050_H_
  #include "MPU6050\MPU6050_6Axis_MotionApps20.h"
  #define _MPU6050_H_
#endif

#ifndef _PID_H_
  #include "pid\pid.h"
  #define _PID_H_
#endif

bool initializeMPU();

void setup()
{
  if (initializeMPU()) { Serial.println("MPU successfully initialized"); }
  Serial.begin(115200);

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop()
{
  // TODO: Read targets from controller
  baseThrottle = 40;

  // TODO: Assign controller targets to target targetYPR
  targetYPR[ARRAY_YAW] = 0;
  targetYPR[ARRAY_PITCH] = 0;
  targetYPR[ARRAY_ROLL] = 0;

  // TODO: Read readYPR values from imu1
  readYPR[ARRAY_YAW] = imu1.getRotationZ();
  readYPR[ARRAY_PITCH] = imu1.getRotationY();
  readYPR[ARRAY_ROLL] = imu1.getRotationX();

  // TODO: PID computations
  pidYaw.computePID(outputYPR[ARRAY_YAW], targetYPR[ARRAY_YAW], readYPR[ARRAY_YAW]);
  pidPitch.computePID(outputYPR[ARRAY_PITCH], targetYPR[ARRAY_PITCH], readYPR[ARRAY_PITCH]);
  pidRoll.computePID(outputYPR[ARRAY_ROLL], targetYPR[ARRAY_ROLL], readYPR[ARRAY_ROLL]);

  // TODO: Fuse PID results into motor outputs
  output_fl = baseThrottle + outputYPR[ARRAY_YAW] + outputYPR[ARRAY_ROLL] + outputYPR[ARRAY_PITCH];
  output_fr = baseThrottle - outputYPR[ARRAY_YAW] - outputYPR[ARRAY_ROLL] + outputYPR[ARRAY_PITCH];
  output_rl = baseThrottle - outputYPR[ARRAY_YAW] + outputYPR[ARRAY_ROLL] - outputYPR[ARRAY_PITCH];
  output_rr = baseThrottle + outputYPR[ARRAY_YAW] - outputYPR[ARRAY_ROLL] - outputYPR[ARRAY_PITCH];

  if (output_fl > MOTOR_OUTPUT_MAX_ABS) { output_fl = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_fl < 0) { output_fl = 0; }
  if (output_fr > MOTOR_OUTPUT_MAX_ABS) { output_fr = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_fr < 0) { output_fr = 0; }
  if (output_rl > MOTOR_OUTPUT_MAX_ABS) { output_rl = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_rl < 0) { output_rl = 0; }
  if (output_rr > MOTOR_OUTPUT_MAX_ABS) { output_rr = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_rr < 0) { output_rr = 0; }

  // TODO: Update motor PWM outputs
  Serial.print(output_fl);
  Serial.print("          ");
  Serial.print(output_fr);
  Serial.println(); Serial.println(); Serial.println(); Serial.println();
  Serial.print(output_rl);
  Serial.print("          ");
  Serial.println(output_rr);
  Serial.println();
  Serial.println();

  analogWrite(3, output_fl);
  analogWrite(5, output_rl);
  analogWrite(9, output_rr);
  analogWrite(10, output_fr);

  delay(20);
}

bool initializeMPU()
{
  imu1.initialize();
  imu1.setXGyroOffset(107);
  imu1.setYGyroOffset(3);
  imu1.setZGyroOffset(25);
  imu1.setZAccelOffset(759);
  imu1.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  return true;
}
