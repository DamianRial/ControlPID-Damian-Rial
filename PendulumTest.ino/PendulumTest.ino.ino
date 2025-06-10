#include <Wire.h>              // Biblioteca para comunicación I2C
#include <MPU6050.h>           // Biblioteca para usar el sensor MPU6050
#include "PIDControl.h"

MPU6050 mpu;                   // Creamos el objeto del sensor MPU6050

// Pines para el motor A
#define AIN1 25
#define AIN2 26
#define PWMA 32

// Pines para el motor B
#define BIN1 27
#define BIN2 14
#define PWMB 33

#define STBY 12                // Pin para activar/desactivar los motores

PIDControl pid(40.0, 10.0, 0.8, 10);  // kp, ki, kd, dt en ms

float angle = 0;                // Ángulo actual del péndulo
float ANGULO_LIMITE = 65.0;    // Cuando el péndulo supera ese ángulo, asumimos que se cae, por lo tanto paramos los motores por seguridad del sistema

unsigned long tiempoPrevio = 0;  // Esta variable sirve para medir el tiempo entre iteraciones

void setup() {
  Serial.begin(115200);       // Estos son los baudios a los que podemos ver lo que imprime el monitor serial si lo tenemos conectado
  Wire.begin(21, 22);         // Definimos los pines SDA y SCL del I2C para medir la aceleración y el ángulo con el MPU6050
  mpu.initialize();           // Iniciamos el sensor MPU6050

  // Definimos los pines de los motores y el STBY como salidas para poder manejar los motores
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);   // Activamos el puente H y ya se activan los motores

  tiempoPrevio = millis();    // Guardamos el tiempo inicial en milisegundos
}

void loop() {
  angle = leerSensorPendulo();

  // Si el ángulo de inclinación medido supera el límite que definimos antes, los motores se paran porque asumimos que se cae (por seguridad)
  if (abs(angle) > ANGULO_LIMITE) {  // El péndulo mide ángulos negativos hacia un lado y positivos hacia el otro, por lo tanto usamos el valor absoluto para coger la inclinación de ambos lados
    pararMotores();
    Serial.println("CAÍDA DETECTADA");  // Imprimimos en el monitor serial que el péndulo se cae
    delay(100);
    return;
  }

  pid.computeIIR(angle);            // Calculamos la salida del PID con el ángulo actual
  float output = pid.getOutput();

  if (abs(pid.getError()) < 1.5) output = 0;  // Si el error es muy pequeño, no hacemos corrección

  int velocidadBase = 110;          // Velocidad inicial a la que se mueven los motores
  if (abs(angle) > 15) velocidadBase += 40;  // Si el ángulo es mayor a 15 grados, aumentamos la velocidad para corregir mejor

  output = constrain(output, -velocidadBase, velocidadBase);  // Limitamos la salida del PID a la velocidad máxima permitida

  aplicarControlMotor(output);      // Aplicamos el control al motor según la salida calculada

  // Mostrar datos por el monitor serial
  Serial.print("Ángulo: ");
  Serial.print(angle);
  Serial.print(" | Salida: ");
  Serial.println(output);

  delay(10);  // Retardo para evitar saturar el procesador
}

float leerSensorPendulo() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Obtener datos del acelerómetro y giroscopio

  // Calcular tiempo transcurrido desde el último ciclo, así podemos estimar cuánto cambió el ángulo en ese tiempo
  unsigned long tiempoActual = millis();
  float dt = (tiempoActual - tiempoPrevio) / 1000.0;
  tiempoPrevio = tiempoActual;

  // Calculamos el ángulo con la fórmula (arctan(y/z) * 180/pi)
  float acelerometroAng = atan2(ay, az) * 180.0 / PI;
  float gyroRate = gx / 131.0;  // Velocidad angular a partir del giroscopio (factor de escala del MPU6050)

  // Combinamos giroscopio (98%) y acelerómetro (2%) para obtener un ángulo más estable
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * acelerometroAng;

  return angle;
}

void aplicarControlMotor(float power) {
  int baseSpeed = constrain(abs(power), 0, 255);
  int speedA = baseSpeed;
  int speedB = baseSpeed;

  // Ajuste de potencia para compensar peso desigual
  if (angle > 0) {
    speedA = baseSpeed * 1.10;  // Si el péndulo se inclina hacia un lado, aumentamos la potencia para estabilizar
    speedB = baseSpeed * 1.00;
  } else {
    speedA = baseSpeed * 1.00;
    speedB = baseSpeed * 1.10;  // Si se inclina hacia el otro lado, aumentamos potencia al otro motor
  }

  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  if (power > 0) {
    // Invertimos el sentido para marcha atrás y corregir la inclinación
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speedA);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speedB);
  } else {
    // Movimiento hacia adelante para corrección contraria
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speedA);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speedB);
  }
}

void pararMotores() {
  // Parar motores para seguridad
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
