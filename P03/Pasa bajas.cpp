#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float pitch = 0;
float roll = 0;

float pitch_f = 0;
float roll_f = 0;

float alpha = 0.2;   // coeficiente del filtro

void setup() {

  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 no encontrado");
    while (1);
  }

  Serial.println("MPU6050 listo");
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float Ax = a.acceleration.x;
  float Ay = a.acceleration.y;
  float Az = a.acceleration.z;

  // Calcular ángulos
  pitch = atan2(-Ax, sqrt(Ay*Ay + Az*Az)) * 180 / PI;
  roll  = atan2(Ay, Az) * 180 / PI;

  // Filtro pasa bajas
  pitch_f = alpha * pitch + (1 - alpha) * pitch_f;
  roll_f  = alpha * roll  + (1 - alpha) * roll_f;

  Serial.print("Pitch: ");
  Serial.print(pitch_f);

  Serial.print("  Roll: ");
  Serial.println(roll_f);

  delay(20);
}

  delay(10);
}
