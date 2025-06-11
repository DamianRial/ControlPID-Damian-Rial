#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {
  public:
    PIDControl(float kp, float ki, float kd, float dt_ms);

    void computeIIR(float measured);
    void computeTiempoDiscreto(float measured);
    void computePasaBaja(float measured);

    float getOutput();
    float getError();

  private:
    float kp, ki, kd;
    float dt;

    // Común a todos
    float output;
    float setpoint;
    float error[3];  // e[t], e[t-1], e[t-2]

    // Para computeTiempoDiscreto
    float integral;
    float previous_error;

    // Para computePasaBaja
    float d0, d1;
    float fd0, fd1;
    float A0, A1;
    float A0d, A1d, A2d;
    float alpha_1, alpha_2;

    bool filtroInicializado;
};

#endif
