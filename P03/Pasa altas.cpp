/*
  Práctica 03. Acelerómetro y Giroscopio:
  Filtro Pasa Altas para sensor MPU6050

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor inercial MPU6050 mediante comunicación I²C
  para medir la orientación del dispositivo en términos de Roll y Pitch. Los datos del
  giroscopio se integran en el tiempo para estimar los ángulos de inclinación. Posteriormente,
  se aplica un filtro pasa altas (High-Pass) que elimina la deriva de baja frecuencia
  acumulada por la integración, conservando únicamente los cambios dinámicos de orientación.
  El resultado se transmite al monitor serial para comparar el comportamiento del ángulo
  integrado frente al ángulo filtrado.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Objeto principal del sensor MPU6050
Adafruit_MPU6050 mpu;

// --- Ángulos integrados desde el giroscopio ---
// Acumulan la orientación estimada por integración numérica
float pitch = 0;
float roll  = 0;

// --- Ángulos filtrados (salida del filtro pasa altas) ---
// Eliminan la componente de deriva de baja frecuencia
float pitchHP = 0;
float rollHP  = 0;

// --- Valores anteriores para el cálculo diferencial del filtro ---
float pitchPrev = 0;
float rollPrev  = 0;

// --- Variables de temporización ---
unsigned long tiempoPrevio; // Marca de tiempo del ciclo anterior (ms)
float dt;                   // Intervalo de tiempo entre muestras (s)

// Frecuencia de corte del filtro pasa altas
// Componentes por debajo de este valor (Hz) serán atenuadas
float fc = 0.1;

void setup()
{
  // Inicializa comunicación serial
  Serial.begin(115200);

  // Inicializa I²C con pines por defecto del ESP32 (SDA = 21, SCL = 22)
  Wire.begin();

  // Inicializa el sensor; se detiene si no responde en el bus I²C
  if (!mpu.begin())
  {
    Serial.println("MPU6050 no encontrado");
    while (1); // Verificar conexiones físicas y dirección I²C (0x68 o 0x69)
  }

  // Registra el tiempo inicial para el primer cálculo de dt
  tiempoPrevio = millis();
}

void loop()
{
  // Contenedores para los eventos del sensor (acelerómetro, giroscopio, temperatura)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcula el intervalo de tiempo transcurrido desde la última muestra
  unsigned long tiempoActual = millis();
  dt = (tiempoActual - tiempoPrevio) / 1000.0; // Conversión ms → s
  tiempoPrevio = tiempoActual;

  // Convierte velocidad angular de rad/s a grados/s
  float gyroX = g.gyro.x * 180.0 / PI;
  float gyroY = g.gyro.y * 180.0 / PI;

  // Integración numérica del giroscopio para estimar ángulos
  // θ[n] = θ[n-1] + ω * dt
  roll  += gyroX * dt;
  pitch += gyroY * dt;

  // --- Parámetros del filtro pasa altas ---
  // Constante de tiempo: τ = 1 / (2π * fc)
  float tau   = 1.0 / (2.0 * PI * fc);
  // Factor de ponderación: alpha = τ / (τ + dt)
  // Valores cercanos a 1 preservan más la señal dinámica
  float alpha = tau / (tau + dt);

  // Filtro pasa altas aplicado a Roll y Pitch
  // y[n] = alpha * (y[n-1] + x[n] - x[n-1])
  rollHP  = alpha * (rollHP  + roll  - rollPrev);
  pitchHP = alpha * (pitchHP + pitch - pitchPrev);

  // Actualiza valores anteriores para el próximo ciclo
  rollPrev  = roll;
  pitchPrev = pitch;

  // Envía los ángulos filtrados al monitor serial
  Serial.print("Roll HP: ");
  Serial.print(rollHP);
  Serial.print("  Pitch HP: ");
  Serial.println(pitchHP);

  // ~100 Hz de muestreo; debe ser coherente con la dinámica del movimiento a detectar
  delay(10);
}
