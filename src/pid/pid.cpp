#include "pid.h"

PID::PID()
{
  Kp = 1.0;
  Ki = 1.0;
  Kd = 1.0;
  prevError = 0.0;
  accumulatedError = 0.0;
  errorRate = 0.0;
}

PID::PID(double P, double I, double D)
{
  Kp = P;
  Ki = I;
  Kd = D;
  prevError = 0.0f;
  accumulatedError = 0.0f;
  errorRate = 0.0f;
}

PID::~PID()
{
  // Empty
}

double PID::setKp(double newKp) { return Kp = newKp; }
double PID::setKi(double newKi) { return Ki = newKi; }
double PID::setKd(double newKd) { return Kd = newKd; }

double PID::getKp() const { return Kp; }
double PID::getKi() const { return Ki; }
double PID::getKd() const { return Kd; }
double PID::getAccumError() const { return accumulatedError; }
double PID::getErrorRate() const { return errorRate; }

bool PID::resetPID()
{
  prevError = 0.0;
  accumulatedError = 0.0f;
  errorRate = 0.0f;
  return true;
}

bool PID::computePID(int16_t& output, int16_t target, int16_t readPos)
{
  int16_t curError = readPos - target;
  accumulatedError += curError;
  errorRate = curError - prevError;

  double output_p = Kp*curError;
  double output_i = Ki*accumulatedError;
  double output_d = Kd*errorRate;

  output = output_p + output_i + output_d;
  return true;
}
