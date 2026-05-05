/*
  Práctica 03. Acelerómetro y Giroscopio:
  Simulador de Bolsa de Aire con sensor MPU6050

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor inercial MPU6050 mediante comunicación I²C
  para simular el sistema de activación de una bolsa de aire (airbag). En cada ciclo se
  calcula la magnitud del vector de aceleración total a partir de sus tres componentes.
  Para distinguir impactos reales de vibraciones cotidianas, se compara la lectura cruda
  contra una línea base suavizada mediante un filtro pasa bajas tipo EMA; si la diferencia
  supera un umbral de impacto predefinido, el sistema considera que ocurrió un choque.
  En ese caso, en lugar de desplegar una bolsa de aire física, se enciende un LED en el
  pin 2 durante un tiempo determinado, simulando la activación del dispositivo de seguridad.
  El estado del sistema se reporta continuamente en el monitor serial.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Objeto principal del sensor MPU6050
Adafruit_MPU6050 mpu;

// Pin del LED que simula la activación de la bolsa de aire
#define PIN_LED 2

// --- Filtro pasa bajas EMA sobre la magnitud de aceleración ---
// Suaviza la señal para obtener una línea base de referencia
float A_filtrado = 0;
// Factor de ponderación (0 < alpha <= 1)
// Valor bajo para que la línea base sea lenta y los picos bruscos resalten
float alpha = 0.1;

// --- Parámetros de detección de impacto ---
// Diferencia mínima entre la aceleración cruda y la línea base para considerar un choque
// Ajustar experimentalmente según la sensibilidad deseada
float umbralImpacto = 15.0; // m/s²

// --- Variables de control del LED ---
bool airbagActivado = false;
unsigned long tiempoActivacion = 0;
// Tiempo que permanece encendido el LED tras detectar el impacto (ms)
unsigned long duracionAirbag = 3000;

void setup()
{
  // Inicializa comunicación serial
  Serial.begin(115200);

  // Configura el pin del LED como salida y lo apaga por defecto
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Inicializa I²C con pines por defecto del ESP32 (SDA = 21, SCL = 22)
  Wire.begin();

  // Inicializa el sensor; se detiene si no responde en el bus I²C
  if (!mpu.begin())
  {
    Serial.println("MPU6050 no encontrado");
    while (1); // Verificar conexiones físicas y dirección I²C (0x68 o 0x69)
  }

  // Rango de ±16g para detectar impactos de alta intensidad
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  Serial.println("Sistema de airbag listo");
}

void loop()
{
  // Contenedores para los eventos del sensor (acelerómetro, giroscopio, temperatura)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcula la magnitud del vector de aceleración total
  // |A| = √(Ax² + Ay² + Az²)
  float A = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  // Actualiza la línea base con el filtro pasa bajas EMA
  // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  A_filtrado = alpha * A + (1 - alpha) * A_filtrado;

  // Calcula la variación brusca respecto a la línea base
  // Un valor alto indica un cambio repentino de aceleración, característico de un impacto
  float delta = A - A_filtrado;

  unsigned long tiempoActual = millis();

  // Detección de impacto: variación repentina supera el umbral y el airbag no está activo
  if (delta > umbralImpacto && !airbagActivado)
  {
    airbagActivado = true;
    tiempoActivacion = tiempoActual;
    digitalWrite(PIN_LED, HIGH); // Simula despliegue de bolsa de aire
    Serial.println("¡IMPACTO DETECTADO! Airbag activado.");
  }

  // Apaga el LED una vez transcurrido el tiempo de activación
  if (airbagActivado && (tiempoActual - tiempoActivacion) > duracionAirbag)
  {
    airbagActivado = false;
    digitalWrite(PIN_LED, LOW);
    Serial.println("Sistema reiniciado. En espera...");
  }

  // Imprime el estado actual del sistema en el monitor serial
  Serial.print("A: ");
  Serial.print(A);
  Serial.print(" m/s²   Base: ");
  Serial.print(A_filtrado);
  Serial.print(" m/s²   Delta: ");
  Serial.print(delta);
  Serial.print(" m/s²   Airbag: ");
  Serial.println(airbagActivado ? "ACTIVO" : "en espera");

  // ~50 Hz de muestreo; suficiente para capturar transientes de impacto
  delay(20);
}
