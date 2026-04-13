#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float pitch = 0;
float roll = 0;

unsigned long tiempoPrevio;
float dt;

void setup() {

  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("No se encontró MPU6050");
    while (1);
  }

  Serial.println("MPU6050 listo");

  tiempoPrevio = millis();
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long tiempoActual = millis();
  dt = (tiempoActual - tiempoPrevio) / 1000.0;
  tiempoPrevio = tiempoActual;

  float gyroX = g.gyro.x * 180 / PI; // convertir a grados/s
  float gyroY = g.gyro.y * 180 / PI;

  roll  += gyroX * dt;
  pitch += gyroY * dt;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("  Pitch: ");
  Serial.println(pitch);

  delay(10);
}
