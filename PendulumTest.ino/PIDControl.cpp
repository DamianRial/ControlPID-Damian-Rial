#include "PIDControl.h"

PIDControl::PIDControl(float kp, float ki, float kd, float dt_ms)
  : kp(kp), ki(ki), kd(kd), dt(dt_ms / 1000.0),
    output(0), setpoint(0), integral(0), previous_error(0),
    d0(0), d1(0), fd0(0), fd1(0), filtroInicializado(false) {
  error[0] = error[1] = error[2] = 0;

  // Coeficientes para computePasaBaja
  A0 = kp + ki * dt;
  A1 = -kp;

  A0d = kd / dt;
  A1d = -2.0 * kd / dt;
  A2d = kd / dt;

  float N = 5.0;
  float tau = kd / (kp * N);
  float alpha = dt / (2 * tau);
  alpha_1 = alpha / (alpha + 1);
  alpha_2 = (alpha - 1) / (alpha + 1);
}

// ------------------ PID IIR --------------------
void PIDControl::computeIIR(float measured) {
  error[2] = error[1];
  error[1] = error[0];
  error[0] = setpoint - measured;

  float a0 = kp + ki * dt + kd / dt;
  float a1 = -kp - 2 * kd / dt;
  float a2 = kd / dt;

  output = a0 * error[0] + a1 * error[1] + a2 * error[2];
}

// ------------------ PID Tiempo Discreto --------------------
void PIDControl::computeTiempoDiscreto(float measured) {
  float e = setpoint - measured;

  integral += e * dt;
  float derivative = (e - previous_error) / dt;
  output = kp * e + ki * integral + kd * derivative;

  previous_error = e;
  error[0] = e;
}

// ------------------ PID con Filtro Pasa Baja --------------------
void PIDControl::computePasaBaja(float measured) {
  error[2] = error[1];
  error[1] = error[0];
  error[0] = setpoint - measured;

  // Parte PI
  output += A0 * error[0] + A1 * error[1];

  // Parte derivativa filtrada
  d1 = d0;
  d0 = A0d * error[0] + A1d * error[1] + A2d * error[2];

  fd1 = fd0;
  fd0 = alpha_1 * (d0 + d1) - alpha_2 * fd1;

  output += fd0;
}

// ------------------ Getters --------------------
float PIDControl::getOutput() {
  return output;
}

float PIDControl::getError() {
  return error[0];
}
