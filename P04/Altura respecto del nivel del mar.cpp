#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

// Presión al nivel del mar estándar (Pa)
// Puedes ajustar este valor con la presión real de tu zona
const float PRESION_NMar = 101325.0;

void setup() {
  Serial.begin(115200);

  if (!bmp.begin(BMP085_ULTRAHIGHRES)) {
    Serial.println("BMP180 no encontrado. Verifica la conexion.");
    while (1);
  }

  for (int i = 0; i < 40; i++) Serial.println();

  Serial.println("  +=========================================+");
  Serial.println("  |     ALTIMETRO BMP180 - Zacatecas        |");
  Serial.println("  |     Presion ref: 101325 Pa (estandar)   |");
  Serial.println("  +=========================================+");
  Serial.println();
}

void loop() {
  float tempC   = bmp.readTemperature();        // °C
  long  presPa  = bmp.readPressure();           // Pa
  float presHPa = presPa / 100.0;              // hPa

  // Fórmula barométrica internacional
  float altitud = 44330.0 * (1.0 - pow((float)presPa / PRESION_NMar, 1.0 / 5.255));

  // Limpiar y redibujar
  for (int i = 0; i < 20; i++) Serial.println();

  Serial.println("  +=========================================+");
  Serial.println("  |          LECTURA DEL SENSOR             |");
  Serial.println("  +=========================================+");
  Serial.printf( "  | Temperatura  : %8.2f  C              |\n", tempC);
  Serial.printf( "  | Presion      : %8ld  Pa             |\n", presPa);
  Serial.printf( "  | Presion      : %8.2f  hPa           |\n", presHPa);
  Serial.println("  +-----------------------------------------+");
  Serial.printf( "  | Altitud      : %8.2f  m s.n.m.      |\n", altitud);
  Serial.println("  +=========================================+");
  Serial.println();
  Serial.println("  Actualizando cada 2 segundos...");

  delay(2000);
}
