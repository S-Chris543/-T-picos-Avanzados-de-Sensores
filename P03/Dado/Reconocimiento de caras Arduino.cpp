/*
  Práctica 03. Acelerómetro y Giroscopio:
  Reconocimiento de Caras de Dado con Red Neuronal y sensor MPU6050

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor inercial MPU6050 mediante comunicación I²C
  para identificar qué cara de un dado físico está orientada hacia arriba. Los ángulos de
  Pitch y Roll calculados a partir del acelerómetro se normalizan y se pasan como entrada
  a una red neuronal de dos capas (perceptrón multicapa) cuyos pesos están almacenados en
  un archivo de cabecera externo. La red devuelve una puntuación de confianza para cada
  una de las seis caras del dado. Con base en esa puntuación y la diferencia entre el
  primer y segundo candidato, el sistema clasifica el estado del dado en tres categorías:
  cara estable, arista (transición entre caras) o posición desconocida. Adicionalmente,
  el programa acumula el tiempo que el dado permanece en cada cara y lo reporta en el
  monitor serial. Un LED indica visualmente el estado del sistema.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Pesos y topología de la red neuronal entrenada externamente
#include "weights_chidos.h"

// Objeto principal del sensor MPU6050
Adafruit_MPU6050 mpu;

// --- Configuración de hardware ---
const uint32_t TS_MS   = 100;  // Período de muestreo en ms (~10 Hz)
const int      I2C_SDA = 8;    // Pin SDA del bus I²C
const int      I2C_SCL = 9;    // Pin SCL del bus I²C
const int      LED_PIN = 21;   // LED de estado: encendido = cara estable, parpadeo = arista

// --- Umbrales de clasificación de la red neuronal ---
// Puntuación mínima del mejor candidato para considerarlo una cara válida
const float FACE_THRESHOLD = 0.50f;
// Diferencia mínima entre el primer y segundo candidato para descartar una arista
// Si la diferencia es menor, el dado está entre dos caras
const float EDGE_MARGIN    = 0.25f;

// --- Contadores de tiempo por cara ---
float    faceAccum[6]  = {0};  // Segundos acumulados por cara (índice 0 = cara 1)
int      activeFace    = -1;   // Cara activa actualmente (-1 = ninguna detectada)
uint32_t faceStartMs   = 0;    // Marca de tiempo de inicio de la cara actual (ms)

// Función de activación de la capa oculta: tangente hiperbólica
// Mapea cualquier valor real al rango (-1, 1)
inline float tansig(float x) { return tanhf(x); }

// Devuelve el índice del segundo mayor valor en el arreglo, excluyendo 'exclude'
// Se usa para identificar la segunda cara más probable según la red neuronal
int secondBest(const float* arr, int n, int exclude) {
  int best = (exclude == 0) ? 1 : 0;
  for (int i = 0; i < n; i++) {
    if (i == exclude) continue;
    if (arr[i] > arr[best]) best = i;
  }
  return best;
}

// Ejecuta la inferencia de la red neuronal de dos capas (MLP)
// Entradas:  vector normalizado [pitch, roll]
// Salidas:   cara principal, su puntuación, cara secundaria, su puntuación y vector completo a2
void neuralPredict(const float* in,
                   int*   face_out,
                   float* score_out,
                   int*   second_face_out,
                   float* second_score_out,
                   float* a2_out)
{
  // Capa oculta: z = W1 * in + b1, a1 = tansig(z)
  float a1[NN_HIDDEN];
  for (int i = 0; i < NN_HIDDEN; i++) {
    float z = b1[i];
    for (int j = 0; j < NN_INPUTS; j++) z += W1[i][j] * in[j];
    a1[i] = tansig(z);
  }

  // Capa de salida: z = W2 * a1 + b2, a2 = tansig(z)
  for (int i = 0; i < NN_OUTPUTS; i++) {
    float z = b2[i];
    for (int j = 0; j < NN_HIDDEN; j++) z += W2[i][j] * a1[j];
    a2_out[i] = tansig(z);
  }

  // Identifica la neurona de salida con mayor activación (mejor cara candidata)
  int best = 0;
  for (int i = 1; i < NN_OUTPUTS; i++)
    if (a2_out[i] > a2_out[best]) best = i;

  int second = secondBest(a2_out, NN_OUTPUTS, best);

  // Convierte índice base-0 a número de cara base-1 (1 a 6)
  *face_out         = best + 1;
  *score_out        = a2_out[best];
  *second_face_out  = second + 1;
  *second_score_out = a2_out[second];
}

// Estados posibles del dado
enum DiceState { STATE_FACE, STATE_EDGE, STATE_UNKNOWN };

// Clasifica el estado del dado según las puntuaciones de la red neuronal
// STATE_FACE:    el dado reposa claramente sobre una cara
// STATE_EDGE:    el dado está en transición entre dos caras (arista)
// STATE_UNKNOWN: la puntuación es demasiado baja para cualquier clasificación
DiceState classifyState(float bestScore, float secondScore,
                        int* edgeFaceA, int* edgeFaceB,
                        int faceA, int faceB)
{
  *edgeFaceA = faceA;
  *edgeFaceB = faceB;

  if (bestScore < FACE_THRESHOLD)               return STATE_UNKNOWN;
  if ((bestScore - secondScore) < EDGE_MARGIN)  return STATE_EDGE;
  return STATE_FACE;
}

// Actualiza el cronómetro de la cara activa y acumula el tiempo por cara
// Recibe la cara detectada (1-6) o -1 si el dado no está en posición estable
// Devuelve los segundos totales acumulados en la cara activa, o -1 si no hay cara
float updateFaceTimer(int detectedFace)
{
  uint32_t now = millis();

  // Si no hay cara válida, congela el contador y guarda lo acumulado
  if (detectedFace < 1 || detectedFace > 6) {
    if (activeFace != -1) {
      faceAccum[activeFace - 1] += (now - faceStartMs) / 1000.0f;
      activeFace = -1;
    }
    return -1.0f;
  }

  // Si cambió la cara, guarda el tiempo de la anterior e inicia la nueva sesión
  if (detectedFace != activeFace) {
    if (activeFace != -1) {
      faceAccum[activeFace - 1] += (now - faceStartMs) / 1000.0f;
    }
    activeFace  = detectedFace;
    faceStartMs = now;
  }

  // Tiempo total = acumulado previo + sesión en curso
  float total = faceAccum[activeFace - 1] + (now - faceStartMs) / 1000.0f;
  return total;
}

void setup()
{
  Serial.begin(115200);

  // Espera al monitor serial con un tiempo límite de 2 s
  uint32_t t_init = millis();
  while (!Serial && (millis() - t_init) < 2000) delay(10);
  delay(500);

  // Configura el LED de estado como salida y lo apaga por defecto
  pinMode(LED_PIN, OUTPUT);

  // Inicializa I²C con los pines definidos para este hardware
  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicializa el sensor; se detiene si no responde en el bus I²C
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 no detectado.");
    while (1) delay(100); // Verificar conexiones físicas y dirección I²C (0x68 o 0x69)
  }

  // Rango ±2g: suficiente para orientación estática del dado
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // Rango ±250°/s para el giroscopio (no se usa en la clasificación, pero se inicializa)
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // Filtro digital de 21 Hz para reducir ruido de alta frecuencia en el acelerómetro
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Imprime la topología de la red neuronal cargada desde weights_chidos.h
  Serial.println("=== Reconocimiento de caras del dado ===");
  Serial.print("Topologia: ");
  Serial.print(NN_INPUTS);  Serial.print("-");
  Serial.print(NN_HIDDEN);  Serial.print("-");
  Serial.println(NN_OUTPUTS);
  Serial.print("FACE_THRESHOLD="); Serial.print(FACE_THRESHOLD, 2);
  Serial.print("  EDGE_MARGIN=");  Serial.println(EDGE_MARGIN, 2);
  delay(500);
}

void loop()
{
  uint32_t t0 = millis();

  // Contenedores para los eventos del sensor (acelerómetro, giroscopio, temperatura)
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Convierte aceleración de m/s² a unidades g para la red neuronal
  float ax = accel.acceleration.x / 9.81f;
  float ay = accel.acceleration.y / 9.81f;
  float az = accel.acceleration.z / 9.81f;

  // Calcula ángulos de orientación a partir del vector de gravedad
  // Pitch: rotación sobre el eje Y — θ = atan2(Ax, √(Ay² + Az²))
  float pitch_deg = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  // Roll:  rotación sobre el eje X — φ = atan2(Ay, Az)
  float roll_deg  = atan2f(ay, az)                   * 180.0f / PI;

  // Discretiza los ángulos a enteros de 10 bits (0-1023) para uniformar el rango
  int pitch_int = constrain((int)((pitch_deg + 90.0f)  * 1023.0f / 180.0f), 0, 1023);
  int roll_int  = constrain((int)((roll_deg  + 180.0f) * 1023.0f / 360.0f), 0, 1023);

  // Normaliza al rango [-1, 1] requerido por la función de activación tansig
  float in[NN_INPUTS];
  in[0] = 2.0f * pitch_int / 1023.0f - 1.0f;
  in[1] = 2.0f * roll_int  / 1023.0f - 1.0f;

  // Ejecuta la inferencia de la red neuronal
  int   face,  secondFace;
  float score, secondScore;
  float a2[NN_OUTPUTS];
  neuralPredict(in, &face, &score, &secondFace, &secondScore, a2);

  // Clasifica el estado del dado según las puntuaciones obtenidas
  int       edgeFaceA, edgeFaceB;
  DiceState state = classifyState(score, secondScore,
                                  &edgeFaceA, &edgeFaceB,
                                  face, secondFace);

  // Solo acumula tiempo si el dado está en una cara estable
  int   faceForTimer  = (state == STATE_FACE) ? face : -1;
  float currentSecs   = updateFaceTimer(faceForTimer);

  // --- Salida serial con ángulos, puntuaciones y estado clasificado ---
  Serial.print("pitch="); Serial.print(pitch_deg, 1);
  Serial.print(" roll=");  Serial.print(roll_deg,  1);
  Serial.print(" | 1ro=C"); Serial.print(face);
  Serial.print("(");        Serial.print(score, 2);
  Serial.print(") 2do=C"); Serial.print(secondFace);
  Serial.print("(");        Serial.print(secondScore, 2);
  Serial.print(") diff="); Serial.print(score - secondScore, 2);
  Serial.print(" -> ");

  switch (state) {
    case STATE_FACE:
      // Cara estable: imprime cara activa, tiempo en ella y acumulados de todas las caras
      Serial.print("CARA "); Serial.print(face);
      Serial.print(" | tiempo en C"); Serial.print(face);
      Serial.print(": "); Serial.print(currentSecs, 1);
      Serial.print("s | [acum] ");
      for (int i = 0; i < 6; i++) {
        float shown = faceAccum[i];
        // Si es la cara activa, suma la sesión en curso al acumulado previo
        if ((i + 1) == activeFace)
          shown += (millis() - faceStartMs) / 1000.0f;
        Serial.print("C"); Serial.print(i + 1);
        Serial.print("="); Serial.print(shown, 1);
        Serial.print("s ");
      }
      Serial.println();
      digitalWrite(LED_PIN, HIGH);   // LED encendido: cara reconocida con certeza
      break;

    case STATE_EDGE:
      // Arista: el dado está entre dos caras, LED parpadea a ~6 Hz
      Serial.print("FUERA DE RANGO (arista C");
      Serial.print(edgeFaceA); Serial.print("-C");
      Serial.print(edgeFaceB); Serial.println(")");
      digitalWrite(LED_PIN, (millis() / 150) % 2);
      break;

    case STATE_UNKNOWN:
      // Posición desconocida: puntuación insuficiente, LED apagado
      Serial.println("FUERA DE RANGO (desconocido)");
      digitalWrite(LED_PIN, LOW);
      break;
  }

  // Espera activa hasta completar el período de muestreo de 100 ms (~10 Hz)
  while (millis() - t0 < TS_MS) {}
}
