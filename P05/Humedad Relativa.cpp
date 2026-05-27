/*
  Práctica 05. Humedad:
  Humedad relativa vs temperatura con tres sensores independientes
  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza tres sensores de temperatura distintos y un sensor
  de humedad relativa para construir una tabla psicométrica en tiempo real. El sensor
  BMP280 provee la temperatura S1 y la presión atmosférica; dos sensores LM35 conectados
  a las entradas analógicas GPIO34 y GPIO35 proveen las temperaturas S2 y S3. La humedad
  relativa se obtiene del AHT20. Cada 3 segundos se imprime una fila en el monitor serial
  con formato de tabla que relaciona %HR contra las tres lecturas de temperatura y la
  presión, cubriendo el rango más amplio posible de humedad relativa.
*/

#include <Wire.h>           // Comunicación I²C para AHT20 y BMP280
#include <Adafruit_AHTX0.h> // Driver del sensor de humedad y temperatura AHT20
#include <Adafruit_BMP280.h> // Driver del sensor de presión y temperatura BMP280

// ─────────────────────────────────────────
//  Pines analógicos para los sensores LM35
//  GPIO34 y GPIO35: pines de solo entrada (input-only),
//  sin función alterna, ideales para ADC sin ruido digital
// ─────────────────────────────────────────
#define LM35_PIN_1  34    // LM35 #1 → GPIO34 (Sensor 2)
#define LM35_PIN_2  35    // LM35 #2 → GPIO35 (Sensor 3)

// Resolución del ADC del ESP32 en modo 12 bits: 0–4095 cuentas
#define ADC_RESOLUCION  4095.0
// Tensión de referencia del ADC; el ESP32 opera internamente a 3.3 V
#define VREF            3.3

// Objetos para manejar los sensores I²C
Adafruit_AHTX0  aht;  // AHT20: fuente de humedad relativa
Adafruit_BMP280 bmp;  // BMP280: fuente de temperatura S1 y presión atmosférica

// Intervalo entre lecturas en milisegundos (~0.33 Hz de muestreo)
const unsigned long INTERVALO = 3000;
unsigned long ultimoTiempo    = 0; // Marca de tiempo de la última lectura
int           filaNro         = 0; // Contador de filas de la tabla

// ─────────────────────────────────────────
//  leerLM35: convierte la señal analógica del LM35 a grados Celsius
//  El LM35 entrega 10 mV/°C → °C = Voltaje × 100
//  Se promedian N muestras consecutivas para reducir el ruido
//  inherente al ADC del ESP32
// ─────────────────────────────────────────
float leerLM35(int pin, int muestras = 10) {
  long suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(pin);
    delay(2); // Pausa mínima entre muestras para estabilizar el ADC
  }
  float adc     = suma / (float)muestras;          // Promedio de cuentas ADC
  float voltaje = (adc / ADC_RESOLUCION) * VREF;   // Conversión cuentas → Voltios
  return voltaje * 100.0;                           // Conversión Voltios → °C
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Bus I²C: SDA = GPIO21, SCL = GPIO22 (pines por defecto ESP32)

  // Configura el ADC en resolución de 12 bits (0–4095 cuentas)
  analogReadResolution(12);

  delay(500);
  Serial.println("\n========================================");
  Serial.println("  TABLA PSICOMÉTRICA - ESP32 WROOM 32D");
  Serial.println("  S1=BMP280  S2=LM35#1  S3=LM35#2");
  Serial.println("========================================\n");

  // --- Inicialización AHT20 ---
  // Dirección I²C fija: 0x38
  if (!aht.begin()) {
    Serial.println("[ERROR] AHT20 no detectado. Verifica la conexion I2C.");
    while (1); // Bloqueo intencional: revisar cableado SDA/SCL y alimentación
  }
  Serial.println("[OK] AHT20 inicializado (humedad).");

  // --- Inicialización BMP280 ---
  // Dirección I²C: 0x77 cuando SDO está conectado a VCC
  if (!bmp.begin(0x77)) {
    Serial.println("[ERROR] BMP280 no detectado en 0x77. Verifica la conexion I2C.");
    while (1); // Bloqueo intencional: revisar cableado SDA/SCL y alimentación
  }
  Serial.println("[OK] BMP280 inicializado (Sensor 1 + presion).");

  // Configuración de oversampling del BMP280:
  // SAMPLING_X2  para temperatura (balance velocidad/precisión)
  // SAMPLING_X16 para presión (máxima resolución disponible)
  // FILTER_X16   filtro IIR para suavizar variaciones bruscas de presión
  // STANDBY_MS_500 pausa interna entre mediciones internas del chip
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // Los pines LM35 no requieren inicialización; solo se confirman en consola
  Serial.println("[OK] LM35 #1 en GPIO34.");
  Serial.println("[OK] LM35 #2 en GPIO35.");

  delay(500);

  // --- Encabezado de la tabla psicométrica ---
  // Se imprime una sola vez al arrancar; las filas de datos se agregan en loop()
  Serial.println();
  Serial.println("N°  | Humedad(%) | S1-BMP(°C)  | S2-LM35(°C) | S3-LM35(°C) | Presion(hPa)");
  Serial.println("----|------------|-------------|-------------|-------------|-------------");
}

void loop() {
  unsigned long ahora = millis(); // Tiempo actual en ms desde el arranque

  // Ejecutar solo cuando haya transcurrido el intervalo de muestreo
  if (ahora - ultimoTiempo >= INTERVALO) {
    ultimoTiempo = ahora;
    filaNro++;

    // --- Lectura de humedad relativa (AHT20) ---
    // getEvent() rellena dos estructuras: humEvt y tempEvt
    // Solo se usa humEvt; la temperatura interna del AHT20 se descarta
    sensors_event_t humEvt, tempEvt;
    aht.getEvent(&humEvt, &tempEvt);
    float humedad = humEvt.relative_humidity; // Humedad relativa en %

    // --- Sensor 1: temperatura del BMP280 ---
    // Temperatura integrada en el BMP280, en grados Celsius
    float tempS1 = bmp.readTemperature();

    // --- Sensores 2 y 3: temperatura de los LM35 ---
    // Cada llamada promedia 10 lecturas ADC para reducir ruido
    float tempS2 = leerLM35(LM35_PIN_1);
    float tempS3 = leerLM35(LM35_PIN_2);

    // --- Presión atmosférica (BMP280) ---
    // readPressure() devuelve Pascales (Pa); se convierte a hPa dividiendo entre 100
    float presion = bmp.readPressure() / 100.0;

    // --- Validación de datos ---
    // Si AHT20 o BMP280 devuelven NaN, se descarta la fila y se reintenta
    if (isnan(humedad) || isnan(tempS1)) {
      Serial.println("[WARN] Lectura invalida, reintentando...");
      return;
    }

    // --- Impresión de fila de la tabla ---
    // snprintf formatea la línea con ancho fijo para mantener la alineación de columnas
    char linea[90];
    snprintf(linea, sizeof(linea),
             "%-3d | %10.2f | %11.2f | %11.2f | %11.2f | %12.2f",
             filaNro, humedad, tempS1, tempS2, tempS3, presion);
    Serial.println(linea);
  }
  // Sin delay() bloqueante: el uso de millis() permite que el ESP32
  // atienda otras tareas internas entre lecturas
}
