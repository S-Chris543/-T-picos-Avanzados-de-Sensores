/*
  Práctica 04. Sensor de presión BMP180:
  Altímetro barométrico con referencia al nivel del mar

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor BMP180 para funcionar como un altímetro
  barométrico. Cada 2 segundos lee la temperatura ambiente y la presión atmosférica,
  luego aplica la Fórmula Barométrica Internacional para estimar la altitud sobre el
  nivel del mar en metros. Los resultados (temperatura, presión en Pa y hPa, y altitud)
  se muestran en el monitor serial con formato de tabla, redibujándose en cada ciclo.
  La presión de referencia al nivel del mar puede ajustarse según la zona geográfica.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Objeto para manejar el sensor BMP180
Adafruit_BMP085 bmp;

// Presión atmosférica estándar al nivel del mar en Pascales (Pa)
// Valor estándar internacional: 101325 Pa = 1 atm = 1013.25 hPa
// Ajustar con la presión real local para mayor precisión en la altitud calculada
const float PRESION_NMar = 101325.0;

void setup()
{
  Serial.begin(115200);

  // Inicializa el BMP180 en modo ultra alta resolución
  if (!bmp.begin(BMP085_ULTRAHIGHRES))
  {
    Serial.println("BMP180 no encontrado. Verifica la conexion.");
    while (1); // Bloqueo intencional: revisar cableado I²C y alimentación
  }

  // Limpia el terminal antes de mostrar el encabezado
  for (int i = 0; i < 40; i++) Serial.println();

  // Encabezado fijo que se muestra una sola vez al arrancar
  Serial.println("  +=========================================+");
  Serial.println("  |     ALTIMETRO BMP180 - Zacatecas        |");
  Serial.println("  |     Presion ref: 101325 Pa (estandar)   |");
  Serial.println("  +=========================================+");
  Serial.println();
}

void loop()
{
  // --- Lectura del sensor ---
  float tempC  = bmp.readTemperature();   // Temperatura en grados Celsius
  long  presPa = bmp.readPressure();      // Presión absoluta en Pascales

  // --- Conversión de unidades ---
  // Convierte Pa a hectopascales (hPa), unidad estándar en meteorología y aviación
  float presHPa = presPa / 100.0;

  // --- Cálculo de altitud ---
  // Fórmula Barométrica Internacional:
  //   h = 44330 * (1 - (P / P0)^(1/5.255))
  // donde P = presión medida (Pa) y P0 = presión al nivel del mar (Pa)
  // El exponente 1/5.255 proviene del modelo de atmósfera estándar
  float altitud = 44330.0 * (1.0 - pow((float)presPa / PRESION_NMar, 1.0 / 5.255));

  // --- Redibujado de pantalla ---
  // Se desplaza el terminal con saltos de línea para simular un refresco de pantalla
  for (int i = 0; i < 20; i++) Serial.println();

  // --- Tabla de resultados ---
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

  // Pausa de 2 segundos entre lecturas (~0.5 Hz de muestreo)
  delay(2000);
}
