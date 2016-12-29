#include <stdint.h>

class PID {
public:

  // Constructors
  PID();
  PID(double P, double I, double D);

  // Destructors
  ~PID();

  // Set control parameters
  double setKp(double newKp);
  double setKi(double newKi);
  double setKd(double newKd);

  // Get control parameters
  double getKp() const;
  double getKi() const;
  double getKd() const;
  double getAccumError() const;   // Error accumulated for integral control
  double getErrorRate() const;    // Difference between current and previous error for derivative control

  // Core PID functionality
  bool resetPID();  // Gain parameters set to 1 by default
  bool computePID(int16_t& output, int16_t target, int16_t readPos);

private:

  double Kp;
  double Ki;
  double Kd;
  float prevError;
  float accumulatedError;
  float errorRate;
};
