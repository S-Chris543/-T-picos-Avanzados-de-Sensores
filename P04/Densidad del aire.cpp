/*
  Práctica 04. Sensor de presión BMP180:
  Medición de presión atmosférica y cálculo de densidad del aire

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor barométrico BMP180 mediante comunicación I²C
  para medir la presión atmosférica y la temperatura ambiente. Con ambos valores aplica la
  Ley del Gas Ideal para calcular la densidad del aire en kg/m³. Los resultados (presión en
  Pa, presión en hPa y densidad) se envían al monitor serial cada 2 segundos, permitiendo
  monitorear las condiciones atmosféricas en tiempo real.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Objeto para manejar el sensor BMP180
// (La librería Adafruit_BMP085 es compatible con BMP180)
Adafruit_BMP085 bmp;

// Constante específica del aire seco (J/kg·K)
// Usada en la Ley del Gas Ideal: P = ρ·R·T → ρ = P / (R·T)
const float R_esp = 287.058;

void setup()
{
  // Inicializa comunicación serial a 115200 baudios
  Serial.begin(115200);

  // Inicializa el sensor en modo de ultra alta resolución
  // Si no se detecta, detiene el programa con mensaje de error
  if (!bmp.begin(BMP085_ULTRAHIGHRES))
  {
    Serial.println("BMP180 no encontrado. Verifica la conexión.");
    while (1); // Bloqueo intencional: revisar cableado I²C y alimentación
  }

  Serial.println("BMP180 listo!\n");
}

void loop()
{
  // --- Lectura del sensor ---

  // Lee la presión absoluta en Pascales (Pa)
  long presPa = bmp.readPressure();

  // Lee la temperatura en grados Celsius
  float tempC = bmp.readTemperature();

  // Convierte temperatura a Kelvin para usar en la Ley del Gas Ideal
  // (la fórmula de densidad requiere temperatura absoluta)
  float tempK = tempC + 273.15;

  // --- Conversión de unidades ---

  // Convierte Pa a hectopascales (hPa) — unidad estándar en meteorología
  float presHPa = presPa / 100.0;

  // --- Cálculo de densidad del aire ---

  // Ley del Gas Ideal: ρ = P / (R_esp · T)
  // P en Pa, T en K → ρ en kg/m³
  float densidad = presPa / (R_esp * tempK);

  // --- Salida serial ---
  Serial.println("==============================");
  Serial.printf("Presión     : %ld Pa  (%.2f hPa)\n", presPa, presHPa);
  Serial.printf("Temperatura : %.2f °C  (%.2f K)\n", tempC, tempK);
  Serial.printf("Densidad    : %.4f kg/m³\n", densidad);
  Serial.println("==============================\n");

  // Espera 2 segundos antes de la siguiente medición (~0.5 Hz de muestreo)
  delay(2000);
}
