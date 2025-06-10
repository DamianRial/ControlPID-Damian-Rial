#include <Wire.h>
#include <MPU6050.h>
#include "PIDControl.h"

MPU6050 mpu;

// Pines para el motor A
#define AIN1 25
#define AIN2 26
#define PWMA 32

// Pines para el motor B
#define BIN1 27
#define BIN2 14
#define PWMB 33

#define STBY 12

PIDControl pid(40.0, 10.0, 0.8, 10);  // kp, ki, kd, dt en ms

float angle = 0;
float ANGULO_LIMITE = 65.0;

unsigned long tiempoPrevio = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  mpu.initialize();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pid.setSetpoint(0.0);

  tiempoPrevio = millis();
}

void loop() {
  angle = leerSensorPendulo();

  if (abs(angle) > ANGULO_LIMITE) {
    pararMotores();
    Serial.println("CAÍDA DETECTADA");
    delay(100);
    return;
  }

  pid.computeIIR(angle);
  float output = pid.getOutput();

  if (abs(pid.getError()) < 1.5) output = 0;

  int velocidadBase = 110;
  if (abs(angle) > 15) velocidadBase += 40;

  output = constrain(output, -velocidadBase, velocidadBase);

  aplicarControlMotor(output);

  Serial.print("Ángulo: ");
  Serial.print(angle);
  Serial.print(" | Salida: ");
  Serial.println(output);

  delay(10);
}

float leerSensorPendulo() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long tiempoActual = millis();
  float dt = (tiempoActual - tiempoPrevio) / 1000.0;
  tiempoPrevio = tiempoActual;

  float acelerometroAng = atan2(ay, az) * 180.0 / PI;
  float gyroRate = gx / 131.0;

  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * acelerometroAng;
  return angle;
}

void aplicarControlMotor(float power) {
  int baseSpeed = constrain(abs(power), 0, 255);
  int speedA = baseSpeed;
  int speedB = baseSpeed;

  if (angle > 0) {
    speedA = baseSpeed * 1.10;
    speedB = baseSpeed * 1.00;
  } else {
    speedA = baseSpeed * 1.00;
    speedB = baseSpeed * 1.10;
  }

  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  if (power > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speedA);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speedB);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speedA);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speedB);
  }
}

void pararMotores() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
