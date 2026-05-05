/*
  Práctica 03. Acelerómetro y Giroscopio:
  Filtro Pasa Bajas para sensor MPU6050

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor inercial MPU6050 mediante comunicación I²C
  para estimar la orientación del dispositivo en términos de Pitch y Roll a partir de los
  datos del acelerómetro. Los ángulos se calculan geométricamente usando la función atan2
  sobre las componentes de aceleración. Para reducir el ruido de alta frecuencia inherente
  al acelerómetro, se aplica un filtro pasa bajas tipo EMA (Exponential Moving Average),
  el cual combina la lectura actual con la anterior filtrada mediante un factor de
  ponderación. Los ángulos filtrados se transmiten continuamente al monitor serial.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Objeto principal del sensor MPU6050
Adafruit_MPU6050 mpu;

// --- Ángulos calculados desde el acelerómetro (sin filtrar) ---
float pitch = 0;
float roll  = 0;

// --- Ángulos filtrados (salida del filtro pasa bajas EMA) ---
float pitch_f = 0;
float roll_f  = 0;

// Factor de ponderación del filtro EMA (0 < alpha <= 1)
// Valores pequeños = más suavizado, mayor atenuación de ruido
float alpha = 0.2;

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

  Serial.println("MPU6050 listo");
}

void loop()
{
  // Contenedores para los eventos del sensor (acelerómetro, giroscopio, temperatura)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Extrae las componentes de aceleración en m/s²
  float Ax = a.acceleration.x;
  float Ay = a.acceleration.y;
  float Az = a.acceleration.z;

  // Calcula ángulos de inclinación a partir del vector de gravedad
  // Pitch: rotación sobre el eje Y — θ = atan2(-Ax, √(Ay² + Az²))
  pitch = atan2(-Ax, sqrt(Ay*Ay + Az*Az)) * 180.0 / PI;
  // Roll: rotación sobre el eje X  — φ = atan2(Ay, Az)
  roll  = atan2(Ay, Az) * 180.0 / PI;

  // Filtro pasa bajas EMA aplicado a Pitch y Roll
  // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  pitch_f = alpha * pitch + (1 - alpha) * pitch_f;
  roll_f  = alpha * roll  + (1 - alpha) * roll_f;

  // Envía los ángulos filtrados al monitor serial
  Serial.print("Pitch: ");
  Serial.print(pitch_f);
  Serial.print("  Roll: ");
  Serial.println(roll_f);

  // ~50 Hz de muestreo; adecuado para movimientos lentos de orientación
  delay(20);
}
