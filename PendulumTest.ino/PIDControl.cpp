#include "PIDControl.h"

PIDControl::PIDControl(float kp, float ki, float kd, float dt_ms) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->dt = dt_ms / 1000.0f; // convertir ms a segundos

  setpoint = 0.0f;
  error = 0.0f;
  prevError = 0.0f;
  errorSum = 0.0f;
  output = 0.0f;
}

void PIDControl::setSetpoint(float sp) {
  setpoint = sp;
  errorSum = 0.0f;  // reiniciar integral al cambiar setpoint
}

void PIDControl::computeIIR(float input) {
  error = setpoint - input;
  errorSum += error * dt;

  // limitar la integral para evitar windup
  if (errorSum > 100) errorSum = 100;
  else if (errorSum < -100) errorSum = -100;

  float dError = (error - prevError) / dt;

  output = kp * error + ki * errorSum + kd * dError;

  prevError = error;
}

float PIDControl::getOutput() const {
  return output;
}

float PIDControl::getError() const {
  return error;
}
