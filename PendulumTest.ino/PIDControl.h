#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <Arduino.h>

class PIDControl {
public:
  PIDControl(float kp, float ki, float kd, float dt_ms);

  void setSetpoint(float sp);

  void computeIIR(float input);
  float getOutput() const;
  float getError() const;

private:
  float kp, ki, kd;
  float dt;

  float setpoint;

  float error;
  float prevError;
  float errorSum;

  float output;
};

#endif
