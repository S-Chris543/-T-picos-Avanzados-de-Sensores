/*
  Proyecto Final. Control por Gestos:
  Control de un carro robótico mediante gestos de inclinación detectados mediante
  un IMU MPU6050, transmitidos vía comunicación inalámbrica ESP-NOW.
  
  Autores:
  - Christian Emmanuel Castruita Alaniz: Desarrollo principal del código.
  - Del Hoyo Gómez Karla Stephanie: Pruebas de funcionamiento y documentación.
  - Pablo David Sánchez García: Apoyo en cálculos y optimización.
  
  Descripción:
  Este programa utiliza un ESP32-S3 junto con un sensor inercial MPU6050 para
  detectar gestos de inclinación (pitch y roll) y traducirlos en comandos de
  movimiento para un carro robótico. El carro recibe los comandos mediante el
  protocolo ESP-NOW, que permite comunicación rápida y de baja latencia sin
  necesidad de un punto de acceso WiFi. El programa calcula los ángulos de
  inclinación a partir de los datos del acelerómetro, los compara contra umbrales
  predefinidos para determinar el gesto dominante (adelante, reversa, giro
  izquierda, giro derecha o quieto), construye un paquete de control con las
  velocidades (PWM) para cada motor, y lo envía cada 50 ms. Un LED indica si el
  último paquete fue confirmado (ACK) por el carro receptor.
*/

// ─────────────────────────────────────────────────────────────────────────────
// CONTROL POR GESTOS — ESP32-S3 + MPU6050
// Transmite gestos al carro vía ESP-NOW.
//
// Gestos:
//   Pitch > +UMBRAL_PITCH  → ADELANTE      (ambos motores = +60)
//   Pitch < -UMBRAL_PITCH  → REVERSA       (ambos motores = -60)
//   Roll  > +UMBRAL_ROLL   → GIRO DERECHA  (izq= 60, der=  0)
//   Roll  < -UMBRAL_ROLL   → GIRO IZQUIERDA(izq=  0, der= 60)
//   Sin umbral             → QUIETO        (ambos = 0)
//
// Giro: un solo motor activo, el opuesto quieto.
//
// LEDs:
//   GPIO21 → LED conectado   (ACK exitoso)
//   GPIO47 → LED desconectado
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── MACs ──────────────────────────────────────────────────────────────────────
// Dirección MAC del ESP32 receptor (el carro). Debe coincidir exactamente con la
// MAC física de esa placa, o ESP-NOW no podrá entregarle los paquetes.
uint8_t macCarro[] = {0xAC, 0xA7, 0x04, 0x26, 0xCB, 0x90};

// ── LEDs ──────────────────────────────────────────────────────────────────────
// Pines de los LEDs indicadores de estado de la conexión ESP-NOW.
#define LED_CONECTADO    21
#define LED_DESCONECTADO 47

// ── I2C / MPU6050 ─────────────────────────────────────────────────────────────
// Pines I2C personalizados del ESP32-S3 para comunicarse con el MPU6050
// (este chip no tiene pines I2C fijos como otros Arduino, hay que declararlos).
#define SDA_PIN 8
#define SCL_PIN 9

// ── Umbrales de gesto (grados) ────────────────────────────────────────────────
// Ángulo mínimo de inclinación, en grados, para considerar que el usuario está
// haciendo un gesto intencional y no un movimiento accidental o ruido del sensor.
const float UMBRAL_PITCH = 30.0f;
const float UMBRAL_ROLL  = 45.0f;

// ── Corrección de orientación del sensor ─────────────────────────────────────
// Multiplicadores de signo para compensar la orientación física en la que el
// MPU6050 quedó montado en el control. Si el sensor está "boca abajo" o rotado
// respecto a como se calculan los ángulos, invertir un eje (-1) corrige el
// sentido del gesto sin tener que cambiar la fórmula de pitch/roll.
const int SIGNO_X = -1;
const int SIGNO_Y = -1;
const int SIGNO_Z = -1;

// ── PWM fijo (máximo 60 en el carro) ─────────────────────────────────────────
// Valor de PWM fijo enviado al carro. No es un valor de 0-255 típico de PWM de
// motores, sino un límite propio de la lógica de control del carro (definida en
// su firmware), que acepta como máximo 60 para no exceder la velocidad segura.
#define PWM_MOVIMIENTO 60   // adelante / reversa — ambos motores
#define PWM_GIRO       60   // giro — solo 1 motor, el otro queda en 0

// ── Intervalo de envío ────────────────────────────────────────────────────────
// Periodo entre envíos de paquetes ESP-NOW. 50 ms (20 Hz) es suficientemente
// rápido para que el control se sienta responsivo sin saturar el canal ESP-NOW
// ni al receptor con tráfico innecesario.
#define INTERVALO_ENVIO_MS 50

// ── Tipos de gesto ────────────────────────────────────────────────────────────
// Enumeración de los gestos posibles. Se transmite como uint8_t dentro del
// paquete ESP-NOW para que el carro sepa qué acción ejecutar sin tener que
// enviar texto ni recalcular ángulos del lado del receptor.
enum Gesto : uint8_t {
    QUIETO         = 0,
    ADELANTE       = 1,
    REVERSA        = 2,
    GIRO_DERECHA   = 3,
    GIRO_IZQUIERDA = 4
};

// ── Paquete ESP-NOW ───────────────────────────────────────────────────────────
// Estructura que se envía por aire al carro. Debe coincidir exactamente (mismos
// tipos y mismo orden de campos) con la estructura que el receptor espera
// decodificar, ya que ESP-NOW transmite los bytes crudos de la struct.
typedef struct {
    uint8_t cmd;       // Gesto detectado (uno de los valores de Gesto)
    int16_t pwm_izq;   // Velocidad/sentido del motor izquierdo
    int16_t pwm_der;   // Velocidad/sentido del motor derecho
} PaqueteControl;

// ── Estado global ─────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
PaqueteControl paquete = {QUIETO, 0, 0};
// Bandera que indica si el último paquete enviado fue confirmado por el carro
// (recibe su valor en el callback de envío, no es información instantánea).
bool conectado = false;
// Marca de tiempo del último envío, usada para espaciar los envíos cada
// INTERVALO_ENVIO_MS sin bloquear el loop con delay().
unsigned long ultimoEnvio = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// Refleja el estado actual de "conectado" en los LEDs físicos: uno encendido
// y el otro apagado, nunca ambos en el mismo estado.
void actualizarLeds() {
    digitalWrite(LED_CONECTADO,    conectado ? HIGH : LOW);
    digitalWrite(LED_DESCONECTADO, conectado ? LOW  : HIGH);
}

// Hace parpadear ambos LEDs alternadamente "veces" ocasiones, con "ms" de
// duración por estado. Se usa como indicador visual de que una etapa de
// inicialización (en este caso, el MPU6050) terminó correctamente.
// Al final deja ambos LEDs apagados; actualizarLeds() se encargará después
// de reflejar el estado real de la conexión ESP-NOW.
void parpadearLeds(int veces, int ms) {
    for (int i = 0; i < veces; i++) {
        digitalWrite(LED_CONECTADO, HIGH); digitalWrite(LED_DESCONECTADO, LOW);  delay(ms);
        digitalWrite(LED_CONECTADO, LOW);  digitalWrite(LED_DESCONECTADO, HIGH); delay(ms);
    }
    digitalWrite(LED_CONECTADO, LOW); digitalWrite(LED_DESCONECTADO, LOW);
}

// Llena la estructura global "paquete" con el comando y las velocidades de
// motor correspondientes al gesto detectado. Esta es la única función que
// decide qué valores de PWM corresponden a cada gesto.
void construirPaquete(Gesto g) {
    paquete.cmd = g;
    switch (g) {
        case ADELANTE:
            // Ambos motores adelante al mismo PWM
            paquete.pwm_izq =  PWM_MOVIMIENTO;
            paquete.pwm_der =  PWM_MOVIMIENTO;
            break;
        case REVERSA:
            // Ambos motores atrás al mismo PWM
            paquete.pwm_izq = -PWM_MOVIMIENTO;
            paquete.pwm_der = -PWM_MOVIMIENTO;
            break;
        case GIRO_DERECHA:
            // Solo el motor IZQUIERDO gira adelante → el carro gira a la derecha
            // Motor derecho queda completamente detenido
            paquete.pwm_izq =  PWM_GIRO;
            paquete.pwm_der =  0;
            break;
        case GIRO_IZQUIERDA:
            // Solo el motor DERECHO gira adelante → el carro gira a la izquierda
            // Motor izquierdo queda completamente detenido
            paquete.pwm_izq =  0;
            paquete.pwm_der =  PWM_GIRO;
            break;
        case QUIETO:
        default:
            paquete.pwm_izq = 0;
            paquete.pwm_der = 0;
            break;
    }
}

// Traduce el enum Gesto a una cadena legible, únicamente para mostrarla en el
// monitor serial durante depuración. No afecta la lógica de control.
const char* nombreGesto(Gesto g) {
    switch (g) {
        case ADELANTE:       return "ADELANTE";
        case REVERSA:        return "REVERSA";
        case GIRO_DERECHA:   return "GIRO DERECHA";
        case GIRO_IZQUIERDA: return "GIRO IZQUIERDA";
        default:             return "QUIETO";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW callback
// ─────────────────────────────────────────────────────────────────────────────

// Se ejecuta automáticamente cada vez que ESP-NOW termina de intentar enviar
// un paquete (ya sea que haya tenido éxito o no). Actualiza la bandera
// "conectado" según el resultado y refleja ese estado en los LEDs de inmediato.
// Nota: el "status" indica si el paquete llegó a la capa de radio/ACK del
// receptor, no necesariamente que el carro lo haya procesado.
void onEnvio(const uint8_t *mac, esp_now_send_status_t status) {
    conectado = (status == ESP_NOW_SEND_SUCCESS);
    actualizarLeds();
}

// ─────────────────────────────────────────────────────────────────────────────
// Inicialización
// ─────────────────────────────────────────────────────────────────────────────

// Configura el ESP32 en modo estación (necesario para ESP-NOW), fija el canal
// WiFi a 6 para asegurar que coincida con el canal del carro receptor, inicia
// ESP-NOW, registra el callback de confirmación de envío y agrega al carro
// como "peer" (par autorizado) para poder enviarle paquetes.
void inicializarESPNOW() {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    esp_now_init();
    esp_now_register_send_cb(onEnvio);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, macCarro, 6);
    peer.channel = 6;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("[ESP-NOW] Iniciado. Canal 6.");
}

// Inicializa el bus I2C en los pines personalizados y configura el MPU6050.
// Si el sensor no responde (mal cableado o dirección I2C incorrecta), el
// programa se detiene indefinidamente en el while(1), ya que sin el sensor
// no hay forma de detectar gestos ni continuar con el resto del programa.
void inicializarMPU() {
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 no encontrado. Revisa el cableado.");
        while (1) delay(10);
    }
    // Rango amplio de acelerómetro y giroscopio para tolerar movimientos
    // bruscos del control sin saturar la lectura.
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // Filtro pasa-bajas para suavizar el ruido de alta frecuencia de las
    // lecturas del acelerómetro antes de calcular los ángulos.
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);

    // Parpadeo de confirmación visual de que el sensor quedó listo.
    parpadearLeds(3, 120);
    Serial.println("[MPU6050] Listo.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup / Loop
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    pinMode(LED_CONECTADO,    OUTPUT);
    pinMode(LED_DESCONECTADO, OUTPUT);
    digitalWrite(LED_CONECTADO,    LOW);
    digitalWrite(LED_DESCONECTADO, HIGH);  // empieza desconectado

    inicializarMPU();
    inicializarESPNOW();

    Serial.println("[LISTO] Control de gestos activo.");
    Serial.println("        Inclina el control para mover el carro.");
}

void loop() {
    // Limita la frecuencia de todo el ciclo de control (lectura + envío) a
    // una vez cada INTERVALO_ENVIO_MS, sin bloquear con delay().
    unsigned long ahora = millis();
    if (ahora - ultimoEnvio < INTERVALO_ENVIO_MS) return;
    ultimoEnvio = ahora;

    // ── Leer MPU6050 ────────────────────────────────────────────────────────
    // Se obtienen aceleración, giroscopio y temperatura en una sola llamada;
    // aquí solo se usa el acelerómetro (a) para calcular la inclinación.
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Aplica la corrección de signo por eje (ver SIGNO_X/Y/Z) a las lecturas
    // crudas del acelerómetro, para que coincidan con la orientación física
    // real del control en la mano del usuario.
    float gx = SIGNO_X * a.acceleration.x;
    float gy = SIGNO_Y * a.acceleration.y;
    float gz = SIGNO_Z * a.acceleration.z;

    // Cálculo de pitch (inclinación adelante/atrás) y roll (inclinación
    // lateral) a partir de los componentes de la gravedad medidos por el
    // acelerómetro, usando la inclinación respecto al eje vertical (Z).
    // El resultado está en grados.
    float pitch = atan2(gx, sqrt(gy * gy + gz * gz)) * 180.0f / PI;
    float roll  = atan2(gy, gz) * 180.0f / PI;

    // ── Detectar gesto ───────────────────────────────────────────────────────
    // Por defecto no hay gesto (carro detenido) hasta que algún ángulo supere
    // su umbral correspondiente.
    Gesto gesto = QUIETO;
    bool pasaPitch = fabsf(pitch) > UMBRAL_PITCH;
    bool pasaRoll  = fabsf(roll)  > UMBRAL_ROLL;

    // El pitch tiene prioridad sobre el roll cuando ambos superan su umbral:
    // solo se interpreta como giro si el roll superó su umbral y, además, su
    // magnitud es mayor que la del pitch. Esto evita que un giro accidental
    // se interprete también como avance/reversa, y viceversa.
    if (pasaPitch && (!pasaRoll || fabsf(pitch) >= fabsf(roll))) {
        gesto = (pitch > 0) ? ADELANTE : REVERSA;
    } else if (pasaRoll) {
        gesto = (roll > 0) ? GIRO_DERECHA : GIRO_IZQUIERDA;
    }

    // ── Construir y enviar paquete ────────────────────────────────────────────
    // Traduce el gesto detectado a velocidades de motor y lo transmite al
    // carro por ESP-NOW. El resultado del envío se conocerá después, de forma
    // asíncrona, en el callback onEnvio().
    construirPaquete(gesto);
    esp_now_send(macCarro, (uint8_t*)&paquete, sizeof(PaqueteControl));

    // ── Debug serial ─────────────────────────────────────────────────────────
    // Línea de diagnóstico con el gesto detectado, los ángulos calculados y
    // los PWM resultantes, útil para calibrar umbrales durante pruebas.
    Serial.printf("[CTRL] %-15s | pitch:%6.1f  roll:%6.1f | izq:%4d  der:%4d\n",
                  nombreGesto(gesto), pitch, roll, paquete.pwm_izq, paquete.pwm_der);
}
