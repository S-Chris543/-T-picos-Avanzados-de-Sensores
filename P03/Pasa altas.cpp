#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float pitch = 0;
float roll = 0;

float pitchHP = 0;
float rollHP = 0;

float pitchPrev = 0;
float rollPrev = 0;

unsigned long tiempoPrevio;
float dt;

float fc = 0.1; // frecuencia de corte (Hz)

void setup() {

  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 no encontrado");
    while (1);
  }

  tiempoPrevio = millis();
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long tiempoActual = millis();
  dt = (tiempoActual - tiempoPrevio) / 1000.0;
  tiempoPrevio = tiempoActual;

  float gyroX = g.gyro.x * 180 / PI;
  float gyroY = g.gyro.y * 180 / PI;

  // Integración del giroscopio
  roll  += gyroX * dt;
  pitch += gyroY * dt;

  // Parámetros del filtro
  float tau = 1.0 / (2 * PI * fc);
  float alpha = tau / (tau + dt);

  // Filtro pasa altas
  rollHP  = alpha * (rollHP  + roll  - rollPrev);
  pitchHP = alpha * (pitchHP + pitch - pitchPrev);

  rollPrev = roll;
  pitchPrev = pitch;

  Serial.print("Roll HP: ");
  Serial.print(rollHP);
  Serial.print("  Pitch HP: ");
  Serial.println(pitchHP);

  delay(10);
}
