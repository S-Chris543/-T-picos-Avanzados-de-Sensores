#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variables del filtro
float A_filtrado = 0;
float alpha = 0.2;   // coeficiente filtro pasa bajas

// Variables de pasos
int pasos = 0;
float umbral = 11.5;   // ajustar experimentalmente

unsigned long ultimoPaso = 0;
unsigned long tiempoMinPaso = 300; // ms

void setup() {

  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("No se encontro el MPU6050");
    while (1);
  }

  Serial.println("MPU6050 listo");
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Magnitud de aceleración
  float A = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  // Filtro pasa bajas
  A_filtrado = alpha * A + (1 - alpha) * A_filtrado;

  unsigned long tiempoActual = millis();

  // Detección de paso
  if (A_filtrado > umbral && (tiempoActual - ultimoPaso) > tiempoMinPaso) {

    pasos++;
    ultimoPaso = tiempoActual;

    Serial.print("Pasos: ");
    Serial.println(pasos);
  }

  delay(20);
}
