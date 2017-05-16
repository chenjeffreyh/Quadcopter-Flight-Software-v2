#include <Arduino.h>
#include <Wire.h>
#include "globals.h"

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

bool initializeMPU();

typedef struct {
  Quaternion q;
  float ypr[3];

} report_data_t;

report_data_t report;

void setup()
{
  pinMode(LED_STATUS_YELLOW, OUTPUT);
  digitalWrite(LED_STATUS_YELLOW, HIGH);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);

  initializeMPU();
  Serial.println(imu1.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP"));
  int8_t devStatus = imu1.dmpInitialize();

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP"));
    imu1.setDMPEnabled(true);
    packetSize = imu1.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_STATUS_RED, OUTPUT);
  pinMode(LED_STATUS_YELLOW, OUTPUT);
  pinMode(LED_STATUS_GREEN, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  receiverVals.throttle = 0;
  receiverVals.yaw = 0;
  receiverVals.pitch = 0;
  receiverVals.roll = 0;
  receiverVals.trim_yaw = 0;
  receiverVals.trim_pitch = 0;
  receiverVals.trim_roll = 0;
  receiverVals.flip = 0;
  receiverVals.highspeed = 0;
  receiverVals.P = 0;
  receiverVals.I = 0;
  receiverVals.D = 0;

  controller.init();

  digitalWrite(LED_STATUS_YELLOW, LOW);
  digitalWrite(LED_STATUS_RED, HIGH);
  digitalWrite(LED_STATUS_YELLOW, HIGH);
  digitalWrite(LED_STATUS_GREEN, HIGH);
  Serial.println("Begin");
  delay(6000);
}

void loop()
{
  int time = micros();

  // TODO: Read targets from controller
  //baseThrottle = 50;
  controller.receive(&receiverVals);
  controller.print(&receiverVals);
  float throttleScale = (receiverVals.throttle - 124.0)/131.0;
  //baseThrottle = (receiverVals.throttle - 124.0)/131.0 * MOTOR_OUTPUT_MAX_ABS;
  //Serial.println("baseThrottle");

  // TODO: Assign controller targets to target targetYPR
  targetYPR[ARRAY_YAW] = 0;
  targetYPR[ARRAY_PITCH] = (receiverVals.pitch - 131.0)/124.0 * 20.0;
  targetYPR[ARRAY_ROLL] = (receiverVals.roll - 128.0)/127.0 * 20.0;
  //targetYPR[ARRAY_ROLL] = 4.;

  // TODO: Read readYPR values from imu1
  /*readYPR[ARRAY_YAW] = imu1.getRotationZ();
  readYPR[ARRAY_PITCH] = imu1.getRotationY();
  readYPR[ARRAY_ROLL] = imu1.getRotationX();*/
  imu1.resetFIFO();
  uint16_t fifoCount =  imu1.getFIFOCount();
  while (fifoCount < packetSize) { fifoCount = imu1.getFIFOCount(); }
  imu1.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  imu1.dmpGetQuaternion(&q, fifoBuffer);
  imu1.dmpGetGravity(&gravity, &q);
  imu1.dmpGetYawPitchRoll(ypr, &q, &gravity);
  readYPR[ARRAY_YAW] = ypr[ARRAY_YAW] * 180/M_PI;
  readYPR[ARRAY_PITCH] = ypr[ARRAY_PITCH] * 180/M_PI;
  readYPR[ARRAY_ROLL] = ypr[ARRAY_ROLL] * 180/M_PI;

  memcpy(&report.q, &q, sizeof(q));
  memcpy(report.ypr, ypr, sizeof(ypr));
  controller.sendGeneral(&report, sizeof(report));
  for( int i = 0; i < 3; i++) {
    Serial.println(ypr[i]);
  }

  // TODO: PID computations
  pidYaw.computePID(outputYPR[ARRAY_YAW], targetYPR[ARRAY_YAW], readYPR[ARRAY_YAW]);
  pidPitch.computePID(outputYPR[ARRAY_PITCH], targetYPR[ARRAY_PITCH], readYPR[ARRAY_PITCH]);
  pidRoll.computePID(outputYPR[ARRAY_ROLL], targetYPR[ARRAY_ROLL], readYPR[ARRAY_ROLL]);

  // TODO: Fuse PID results into motor outputs
  baseThrottle = 100 * throttleScale;
  output_fr = baseThrottle - outputYPR[ARRAY_YAW] + outputYPR[ARRAY_ROLL] + outputYPR[ARRAY_PITCH];
  output_rl = baseThrottle - outputYPR[ARRAY_YAW] - outputYPR[ARRAY_ROLL] - outputYPR[ARRAY_PITCH];
  output_rr = baseThrottle + outputYPR[ARRAY_YAW] + outputYPR[ARRAY_ROLL] - outputYPR[ARRAY_PITCH];
  output_fl = baseThrottle + outputYPR[ARRAY_YAW] - outputYPR[ARRAY_ROLL] + outputYPR[ARRAY_PITCH];

  output_fr *= throttleScale;
  output_rl *= throttleScale;
  output_rr *= throttleScale;
  output_fl *= throttleScale;

  if (output_fl > MOTOR_OUTPUT_MAX_ABS) { output_fl = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_fl < 0) { output_fl = 0; }
  if (output_fr > MOTOR_OUTPUT_MAX_ABS) { output_fr = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_fr < 0) { output_fr = 0; }
  if (output_rl > MOTOR_OUTPUT_MAX_ABS) { output_rl = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_rl < 0) { output_rl = 0; }
  if (output_rr > MOTOR_OUTPUT_MAX_ABS) { output_rr = MOTOR_OUTPUT_MAX_ABS; }
  else if (output_rr < 0) { output_rr = 0; }

  // TODO: Update motor PWM outputs
  /*
  Serial.print(readYPR[ARRAY_YAW]);
  Serial.print(" ");
  Serial.print(readYPR[ARRAY_PITCH]);
  Serial.print(" ");
  Serial.println(readYPR[ARRAY_ROLL]);
  Serial.print(output_fl);
  Serial.print("          ");
  Serial.print(output_fr);
  Serial.println(); Serial.println(); Serial.println(); Serial.println();
  Serial.print(output_rl);
  Serial.print("          ");
  Serial.println(output_rr);
  Serial.println();
  Serial.println();
  */

  analogWrite(3, output_fl);
  analogWrite(5, output_rl);
  analogWrite(9, output_rr);
  analogWrite(10, output_fr);

  while ((micros() - time) < 8000) {}

}

bool initializeMPU()
{
  imu1.initialize();
  imu1.setXGyroOffset(98);
  imu1.setYGyroOffset(7);
  imu1.setZGyroOffset(25);
  imu1.setXAccelOffset(-15);
  imu1.setYAccelOffset(-1415);
  imu1.setZAccelOffset(789);
  imu1.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  return true;
}
