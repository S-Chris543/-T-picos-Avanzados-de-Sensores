#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

// Constante específica del aire seco (J/kg·K)
const float R_esp = 287.058;

void setup() {
  Serial.begin(115200);

  if (!bmp.begin(BMP085_ULTRAHIGHRES)) {
    Serial.println("BMP180 no encontrado. Verifica la conexión.");
    while (1);
  }

  Serial.println("BMP180 listo!\n");
}

void loop() {
  // --- Lectura del sensor ---
  long  presPa = bmp.readPressure();        // Pa

  // --- Conversion ---
  float presHPa = presPa / 100.0;           // hPa (solo para mostrar)

  // --- Cálculo de densidad ---
  float densidad = presPa / (R_esp * tempK); // kg/m³

  // --- Salida serial ---
  Serial.println("==============================");
  Serial.printf("Presión     : %ld Pa  (%.2f hPa)\n", presPa, presHPa);
  Serial.printf("Densidad    : %.4f kg/m³\n", densidad);
  Serial.println("==============================\n");

  delay(2000);
}
