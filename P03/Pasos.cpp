/*
  Práctica 03. Acelerómetro y Giroscopio:
  Contador de Pasos con sensor MPU6050

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor inercial MPU6050 mediante comunicación I²C
  para implementar un podómetro básico. En cada ciclo se calcula la magnitud del vector
  de aceleración total a partir de sus tres componentes. Dicha magnitud se suaviza con un
  filtro pasa bajas tipo EMA para eliminar el ruido de alta frecuencia del sensor. La
  detección de un paso ocurre cuando la magnitud filtrada supera un umbral predefinido y
  ha transcurrido un tiempo mínimo desde el último paso registrado, evitando conteos
  duplicados por una misma pisada. El conteo acumulado se reporta en el monitor serial
  cada vez que se detecta un nuevo paso.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Objeto principal del sensor MPU6050
Adafruit_MPU6050 mpu;

// --- Filtro pasa bajas EMA sobre la magnitud de aceleración ---
float A_filtrado = 0;
// Factor de ponderación (0 < alpha <= 1)
// Valores pequeños = más suavizado, menor sensibilidad a picos bruscos
float alpha = 0.2;

// --- Variables del contador de pasos ---
int pasos = 0;

// Magnitud mínima de aceleración (m/s²) para considerar un paso
// Ajustar experimentalmente según el dispositivo y la persona
float umbral = 11.5;

// Marca de tiempo del último paso detectado (ms)
unsigned long ultimoPaso = 0;

// Tiempo mínimo entre pasos consecutivos para evitar conteos duplicados (ms)
// ~300 ms equivale a un máximo de ~200 pasos/min, razonable para caminata/trote
unsigned long tiempoMinPaso = 300;

void setup()
{
  // Inicializa comunicación serial
  Serial.begin(115200);

  // Inicializa I²C con pines por defecto del ESP32 (SDA = 21, SCL = 22)
  Wire.begin();

  // Inicializa el sensor; se detiene si no responde en el bus I²C
  if (!mpu.begin())
  {
    Serial.println("No se encontro el MPU6050");
    while (1); // Verificar conexiones físicas y dirección I²C (0x68 o 0x69)
  }

  Serial.println("MPU6050 listo");
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

  // Filtro pasa bajas EMA para suavizar la señal de aceleración
  // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  A_filtrado = alpha * A + (1 - alpha) * A_filtrado;

  unsigned long tiempoActual = millis();

  // Detección de paso: se requiere superar el umbral Y respetar el tiempo mínimo entre pasos
  if (A_filtrado > umbral && (tiempoActual - ultimoPaso) > tiempoMinPaso)
  {
    pasos++;
    ultimoPaso = tiempoActual;
    Serial.print("Pasos: ");
    Serial.println(pasos);
  }

  // ~50 Hz de muestreo; suficiente para capturar el impacto de cada paso
  delay(20);
}
