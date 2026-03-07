/*
Práctica 02. Interferometría:
Reconstrucción Geométrica
  
- Christian Emmanuel Castruita Alaniz
- Del Hoyo Gómez Karla Stephanie
- Pablo David Sánchez García


Descripción:
Esté código nos permite medir continuamente la distancia con el sensor VL53L0X y, suponiendo que el objeto frente al 
sensor gira con velocidad angular lo más constante posible, convierte cada medición de distancia en coordenadas (x, y) 
usando  ecuaciones trigonométricas. Estos puntos se envían por el puerto serial para poder reconstruir la forma del objeto en una gráfica 2D.
*/

#include <Wire.h>      
#include <VL53L0X.h>   

VL53L0X sensor;       


#define SDA_PIN 21
#define SCL_PIN 22

// Periodo de rotación del objeto (tiempo que tarda en dar una vuelta completa)
const float periodo = 6.0; // segundos por vuelta

// Velocidad angular calculada a partir del periodo
// ω = 2π / T
const float omega = 2 * PI / periodo;

// Variable para guardar el tiempo inicial del experimento
unsigned long tiempoInicio;

void setup() {

  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  // Tiempo máximo de espera para una medición del sensor
  sensor.setTimeout(500);

  // Inicializa el sensor VL53L0X
  if (!sensor.init()) {
    Serial.println("Error VL53L0X"); 
    while (1);                
  }

  // Configura el tiempo de medición del sensor (mejor precisión)
  sensor.setMeasurementTimingBudget(20000);

  // Activa el modo de medición continua del sensor
  sensor.startContinuous();

  // Guarda el tiempo inicial del sistema
  tiempoInicio = millis();
}

void loop() {

  // Calcula el tiempo transcurrido desde el inicio (en segundos)
  float t = (millis() - tiempoInicio) / 1000.0;

  // Calcula el ángulo del objeto suponiendo velocidad angular constante
  // θ = ωt
  float theta = omega * t;

  // Lee la distancia medida por el sensor en milímetros
  uint16_t d = sensor.readRangeContinuousMillimeters();

  // Verifica que no haya ocurrido un timeout en la medición
  if (!sensor.timeoutOccurred()) {

    // Conversión de coordenadas polares a coordenadas cartesianas
    // x = r cos(θ)
    float x = d * cos(theta);

    // y = r sin(θ)
    float y = d * sin(theta);

    // Envía las coordenadas al monitor serial separadas por coma
    // Esto permite graficarlas fácilmente en MATLAB
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
  }

  // Pequeño retardo para controlar la frecuencia de muestreo
  delay(40);
}
