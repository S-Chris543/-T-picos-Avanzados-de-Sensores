/*
Práctica 02. Interferometría:
Filtro EMA para sensor VL53L0X
  
- Christian Emmanuel Castruita Alaniz
- Del Hoyo Gómez Karla Stephanie
- Pablo David Sánchez García


Descripción:
Este programa para ESP32 utiliza el sensor de distancia VL53L0X mediante comunicación I²C para medir continuamente la distancia a un objeto. 
Cada medición obtenida se procesa con un filtro EMA (Exponential Moving Average), el cual suaviza las variaciones y reduce el ruido típico 
del sensor al combinar la lectura actual con la anterior filtrada mediante un factor de ponderación. Finalmente, el código envía al monitor 
serial tanto la distancia original como la distancia filtrada, permitiendo observar cómo el filtro mejora la estabilidad de las mediciones.
*/
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// Parámetro del filtro EMA (0 < alpha <= 1)
// valores pequeños = más suavizado
float alpha = 0.2;

// Variable para guardar el valor filtrado
float distanciaFiltrada = 0;

// Variable para saber si es la primera medición
bool primeraLectura = true;

void setup()
{
  // Inicializa comunicación serial
  Serial.begin(115200);

  // Inicializa I2C del ESP32
  // Pines estándar SDA = 21, SCL = 22
  Wire.begin(21, 22);

  // Tiempo máximo de espera del sensor
  sensor.setTimeout(500);

  // Inicializa el sensor
  if (!sensor.init())
  {
    Serial.println("Error al detectar el VL53L0X");
    while (1); // Se detiene si no encuentra el sensor
  }

  // Ajusta el tiempo de medición (mejor precisión)
  sensor.setMeasurementTimingBudget(20000);

  // Activa medición continua
  sensor.startContinuous();

  Serial.println("Sensor iniciado");
}

void loop()
{
  // Lee la distancia en milímetros
  uint16_t distancia = sensor.readRangeContinuousMillimeters();

  // Verifica si hubo timeout
  if (sensor.timeoutOccurred())
  {
    Serial.println("Timeout del sensor");
    return;
  }

  // Si es la primera lectura, inicializa el filtro
  if (primeraLectura)
  {
    distanciaFiltrada = distancia;
    primeraLectura = false;
  }
  else
  {
    // Filtro EMA
    // y[n] = alpha*x[n] + (1-alpha)*y[n-1]
    distanciaFiltrada = alpha * distancia + (1 - alpha) * distanciaFiltrada;
  }

  // Imprime valores
  Serial.print("Distancia cruda: ");
  Serial.print(distancia);
  Serial.print(" mm   ");

  Serial.print("Distancia filtrada: ");
  Serial.print(distanciaFiltrada);
  Serial.println(" mm");

  delay(50);
}
