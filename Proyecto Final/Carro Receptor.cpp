/*
  Proyecto Final. Carro Receptor:
  Recepción de comandos de control por gestos vía ESP-NOW, con evasión autónoma
  de obstáculos mediante sensor de distancia VL53L0X y giros controlados por PID
  angular usando el giroscopio del MPU6050.
 
  Autores:
  - Christian Emmanuel Castruita Alaniz: Desarrollo principal del código.
  - Del Hoyo Gómez Karla Stephanie: Pruebas de funcionamiento y documentación.
  - Pablo David Sánchez García: Apoyo en cálculos y optimización.

  Descripción:
  Este programa corre en el ESP32-S3 montado sobre el carro robótico. Por un
  lado, actúa como receptor ESP-NOW: recibe paquetes de control enviados por el
  control de gestos (otro ESP32-S3) y los traduce directamente en velocidades de
  PWM para los dos motores mediante un puente H. Por otro lado, monitorea
  constantemente la distancia frente al carro con un sensor VL53L0X; si detecta
  un obstáculo a 8 cm o menos, interrumpe el control manual y ejecuta una
  secuencia de evasión autónoma y bloqueante: retrocede, gira 90° a la derecha,
  avanza, gira 90° a la izquierda y avanza de nuevo, para luego devolver el
  control al usuario. Los giros de 90° no se hacen por tiempo fijo, sino que se
  controlan con un lazo PID que integra la velocidad angular medida por el
  giroscopio del MPU6050, logrando giros más precisos y repetibles.
*/

// ─────────────────────────────────────────────────────────────────────────────
// CARRO — Receptor ESP-NOW + control por gestos + VL53L0X + evasión con PID
//          angular usando MPU6050 (Adafruit_MPU6050)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── MAC del control ───────────────────────────────────────────────────────────
// Dirección MAC del ESP32 transmisor (el control de gestos). ESP-NOW la usa
// para registrar el "peer" autorizado del cual se reciben paquetes.
uint8_t macControl[] = {0xAC, 0xA7, 0x04, 0x26, 0xCB, 0x14};

// ── Pines físicos ─────────────────────────────────────────────────────────────
// Pines GPIO conectados directamente a las entradas del puente H (driver de
// motores). Los nombres "IN1/IN2" siguen la nomenclatura típica de un puente H
// (p. ej. L298N/BTS7960): cada motor se controla con un par de entradas.
#define PIN_IN1MR 15
#define PIN_IN1ML 14
#define PIN_IN2ML 13
#define PIN_IN2MR 12

// ── Mapeo lógico ──────────────────────────────────────────────────────────────
// Renombra los pines físicos a su función lógica (RPWM/LPWM por motor), para
// que el resto del código no tenga que recordar a qué IN1/IN2 corresponde cada
// sentido de giro. RPWM = sentido "adelante", LPWM = sentido "atrás" del motor.
#define PIN_RPWM_IZQ PIN_IN1MR   // GPIO15
#define PIN_LPWM_IZQ PIN_IN2MR   // GPIO12
#define PIN_RPWM_DER PIN_IN2ML   // GPIO13
#define PIN_LPWM_DER PIN_IN1ML   // GPIO14

// ── LEDC ──────────────────────────────────────────────────────────────────────
// Configuración del periférico LEDC del ESP32 (generador de PWM por hardware).
// 1 kHz es una frecuencia típica para drivers de motor DC; 8 bits de resolución
// da un rango de PWM de 0 a 255.
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// Canales LEDC, uno por cada entrada de PWM de los motores (4 en total, ya que
// cada motor necesita dos canales para poder girar en ambos sentidos).
#define CH_RPWM_IZQ 0
#define CH_LPWM_IZQ 1
#define CH_RPWM_DER 2
#define CH_LPWM_DER 3

// ── Límites y tiempos ─────────────────────────────────────────────────────────
// PWM_MAX_CARRO no se usa directamente para limitar valores en este archivo
// (los PWM ya vienen acotados a -255..255 dentro de motorIzquierdo/Derecho),
// pero documenta el límite de velocidad "seguro" recomendado para el carro.
#define PWM_MAX_CARRO         40
// Tiempo máximo sin recibir un paquete del control antes de detener los
// motores por seguridad (evita que el carro siga moviéndose si se pierde la
// señal ESP-NOW).
#define TIMEOUT_MS            300
// Periodo entre lecturas del sensor de distancia, para no saturar el bus I2C
// ni el loop con mediciones innecesariamente frecuentes.
#define SENSOR_INTERVALO_MS   100
// Distancia mínima frente al carro, en milímetros, que dispara la secuencia
// de evasión automática.
#define DISTANCIA_EVASION_MM  80    // 8 cm en milímetros

// ── Parámetros de la secuencia de evasión ─────────────────────────────────────
// PWM y duración (en ms) de cada tramo de movimiento recto dentro de la
// secuencia de evasión. Estos tramos se hacen por tiempo fijo (no por PID),
// a diferencia de los giros de 90°.
#define EVASION_RETROCESO_PWM     -45
#define EVASION_RETROCESO_MS      450
#define EVASION_AVANCE_PWM         45
#define EVASION_AVANCE_MS         350

// Ángulos objetivo de giro durante la evasión, en grados, según la convención
// de signo del PID angular (ver girarPID): positivo = horario/derecha,
// negativo = antihorario/izquierda.
#define GIRO_ANGULO_DER            90.0f   // grados, sentido horario (derecha)
#define GIRO_ANGULO_IZQ           -90.0f   // grados, sentido antihorario (izquierda)
#define GIRO_TOLERANCIA_DEG         2.0f   // tolerancia de llegada
#define GIRO_PWM_MAX               55      // PWM máximo durante el giro
#define GIRO_PWM_MIN_EFECTIVO      18      // PWM mínimo para que el motor realmente gire
#define GIRO_TIMEOUT_MS           2500     // seguridad: si no llega, corta igual
#define GIRO_ESTABLE_MS            120     // tiempo dentro de tolerancia para confirmar

// ── PID angular ───────────────────────────────────────────────────────────────
// Constantes del controlador PID usado para los giros de 90°. Se ajustaron
// experimentalmente: Kp domina la respuesta, Ki corrige error residual lento,
// Kd amortigua el sobreimpulso cerca del objetivo.
#define PID_KP   2.2f
#define PID_KI   0.02f
#define PID_KD   0.6f
// Límite anti-windup: evita que el término integral crezca sin control
// mientras el error es grande y persistente (por ejemplo, al inicio del giro).
#define PID_INTEGRAL_MAX 30.0f

// ── Paquete ESP-NOW ───────────────────────────────────────────────────────────
// Debe coincidir exactamente (mismos tipos y orden) con la estructura definida
// en el control de gestos, ya que ESP-NOW transmite los bytes crudos del struct.
typedef struct {
    uint8_t cmd;
    int16_t pwm_izq;
    int16_t pwm_der;
} PaqueteControl;

// ── Estado global ─────────────────────────────────────────────────────────────
// Marca de tiempo del último paquete válido recibido por ESP-NOW, usada para
// el timeout de seguridad de los motores.
unsigned long ultimoPaquete  = 0;
// Marca de tiempo de la última lectura del sensor de distancia.
unsigned long ultimaSensor   = 0;
// Indica si los motores están actualmente respondiendo a un comando recibido
// (se usa junto con TIMEOUT_MS para saber cuándo aplicar el corte automático).
bool motorActivo             = false;
// Bandera que indica que la secuencia de evasión está en curso. Mientras esté
// activa, se ignoran tanto los paquetes ESP-NOW entrantes como las nuevas
// lecturas del sensor de distancia, dándole control exclusivo de los motores.
bool enEvasion               = false;   // bloquea ESP-NOW durante la secuencia

// ── Sensores ──────────────────────────────────────────────────────────────────
Adafruit_VL53L0X sensorDistancia = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
// Bias (offset) del giroscopio en el eje Z, medido en reposo durante el setup.
// Todo giroscopio MEMS tiene un pequeño error sistemático en su lectura aun
// estando quieto; restar este offset es necesario para que la integración del
// ángulo (yaw) en girarPID() no se desvíe con el tiempo.
float offsetGyroZ = 0.0f;   // bias medido en reposo (rad/s)

// ─────────────────────────────────────────────────────────────────────────────
// Control de motores
// ─────────────────────────────────────────────────────────────────────────────

// Controla el motor izquierdo mediante los dos canales PWM (RPWM/LPWM) del
// puente H. El signo de "v" determina el sentido: positivo activa RPWM
// (adelante), negativo activa LPWM (atrás) usando el valor absoluto como
// magnitud, y 0 deja ambos canales apagados (motor en "freno"/inercia, según
// el driver). Nunca se activan ambos canales a la vez, para no generar un
// cortocircuito en el puente H.
void motorIzquierdo(int v) {
    v = constrain(v, -255, 255);
    if (v > 0) {
        ledcWrite(CH_RPWM_IZQ, v);
        ledcWrite(CH_LPWM_IZQ, 0);
    } else if (v < 0) {
        ledcWrite(CH_RPWM_IZQ, 0);
        ledcWrite(CH_LPWM_IZQ, -v);
    } else {
        ledcWrite(CH_RPWM_IZQ, 0);
        ledcWrite(CH_LPWM_IZQ, 0);
    }
}

// Misma lógica que motorIzquierdo(), pero para el motor derecho.
void motorDerecho(int v) {
    v = constrain(v, -255, 255);
    if (v > 0) {
        ledcWrite(CH_RPWM_DER, v);
        ledcWrite(CH_LPWM_DER, 0);
    } else if (v < 0) {
        ledcWrite(CH_RPWM_DER, 0);
        ledcWrite(CH_LPWM_DER, -v);
    } else {
        ledcWrite(CH_RPWM_DER, 0);
        ledcWrite(CH_LPWM_DER, 0);
    }
}

// Detiene ambos motores. Se llama tras cada tramo de movimiento y ante
// condiciones de seguridad (timeout, fin de evasión, etc.).
void detenerTodo() {
    motorIzquierdo(0);
    motorDerecho(0);
}

// ─────────────────────────────────────────────────────────────────────────────
// MPU6050 — calibración y lectura de velocidad angular en Z
// ─────────────────────────────────────────────────────────────────────────────

// Mide el bias del giroscopio en el eje Z promediando N lecturas con el carro
// en reposo (debe ejecutarse una sola vez, en el setup, antes de mover el
// carro). El offset resultante se resta en cada lectura posterior para que
// la integral del ángulo no se desvíe ("drift") por error sistemático del
// sensor incluso cuando no hay rotación real.
void calibrarGyroZ() {
    Serial.println("[MPU] Calibrando giroscopio (no mover el carro)...");
    const int N = 200;
    double suma = 0.0;
    sensors_event_t a, g, temp;

    for (int i = 0; i < N; i++) {
        mpu.getEvent(&a, &g, &temp);
        suma += g.gyro.z;
        delay(5);
    }
    offsetGyroZ = suma / N;
    Serial.printf("[MPU] Offset gyro Z: %.5f rad/s\n", offsetGyroZ);
}

// Devuelve velocidad angular en Z corregida, en grados/segundo
// (la librería entrega el giroscopio en rad/s; aquí se le resta el offset
// calculado en calibrarGyroZ() y se convierte a deg/s para trabajar en las
// mismas unidades que GIRO_ANGULO_DER/IZQ y GIRO_TOLERANCIA_DEG).
float leerGyroZ_degps() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float gz_rad = g.gyro.z - offsetGyroZ;
    return gz_rad * 180.0f / PI;
}

// ─────────────────────────────────────────────────────────────────────────────
// Giro controlado por PID angular usando el MPU6050
// objetivoDeg > 0 → giro horario (derecha) | < 0 → giro antihorario (izquierda)
// ─────────────────────────────────────────────────────────────────────────────

// Ejecuta un giro hasta alcanzar "objetivoDeg" grados respecto al ángulo en
// que empezó el giro (no respecto a un norte absoluto), integrando la
// velocidad angular del giroscopio en tiempo real y corrigiendo el PWM
// diferencial de los motores mediante un controlador PID. Es una función
// bloqueante: no retorna hasta llegar al objetivo, alcanzar el timeout de
// seguridad, o (en teoría) nunca, si ninguna de esas condiciones se cumpliera.
void girarPID(float objetivoDeg) {
    Serial.printf("[GIRO] Objetivo: %.1f deg\n", objetivoDeg);

    // "yaw" es el ángulo girado acumulado desde el inicio de esta llamada,
    // calculado por integración numérica de la velocidad angular (no viene
    // de un sensor de orientación absoluta, por lo que también acumula
    // pequeños errores de integración además del ruido del giroscopio).
    float yaw = 0.0f;            // ángulo acumulado, relativo al inicio del giro
    float errorPrev = objetivoDeg - yaw;
    float integral = 0.0f;

    // "ultimoT" se usa para calcular el dt real entre iteraciones (necesario
    // para integrar correctamente tanto el yaw como los términos I/D del PID,
    // ya que el bucle no tiene una duración fija por la presencia de delay(5)
    // y tiempos de lectura I2C variables).
    unsigned long ultimoT = micros();
    unsigned long inicioGiro = millis();
    unsigned long tEstable = 0;
    bool dentroDeTolerancia = false;

    while (true) {
        unsigned long ahora = millis();

        // ── Seguridad: timeout del giro ──────────────────────────────────────
        // Si el giro no converge dentro de GIRO_TIMEOUT_MS (por ejemplo, el
        // carro está atascado o las ganancias del PID no logran estabilizar),
        // se aborta el giro en lugar de quedar bloqueado indefinidamente.
        if (ahora - inicioGiro > GIRO_TIMEOUT_MS) {
            Serial.println("[GIRO] Timeout de seguridad alcanzado.");
            break;
        }

        // ── Integración del ángulo (dt real) ─────────────────────────────────
        unsigned long tAhora = micros();
        float dt = (tAhora - ultimoT) / 1000000.0f;
        ultimoT = tAhora;
        // Protección ante overflow de micros() (ocurre cada ~70 min) o ante un
        // dt anormalmente grande (por algún bloqueo momentáneo del bucle):
        // se sustituye por un valor de respaldo razonable en vez de propagar
        // un dt inválido o negativo al resto del cálculo.
        if (dt <= 0 || dt > 0.5f) dt = 0.01f;  // protección ante overflow/lag

        float gz_degps = leerGyroZ_degps();
        yaw += gz_degps * dt;   // integración: ángulo += velocidad angular × dt

        // ── PID ───────────────────────────────────────────────────────────────
        // Error = cuánto le falta al carro para llegar al ángulo objetivo.
        float error = objetivoDeg - yaw;

        // Término integral: acumula el error en el tiempo para corregir
        // desviaciones pequeñas y persistentes; se limita (anti-windup) para
        // que no se dispare mientras el error inicial es grande.
        integral += error * dt;
        integral = constrain(integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

        // Término derivativo: estima qué tan rápido cambia el error, para
        // amortiguar el movimiento antes de que se pase del objetivo.
        float derivada = (error - errorPrev) / dt;
        errorPrev = error;

        // Salida del PID: combinación ponderada de los tres términos.
        float salida = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivada);

        // ── Conversión a PWM diferencial (giro sobre el propio eje) ───────────
        // Se limita la salida del PID al rango de PWM permitido durante el giro.
        int pwm = (int)constrain(salida, -GIRO_PWM_MAX, GIRO_PWM_MAX);

        // Evita el "zumbido sin movimiento" cuando el PWM es demasiado bajo
        // Si el PID pide un PWM distinto de cero pero demasiado pequeño para
        // vencer la fricción/inercia del motor, se eleva a un mínimo efectivo
        // (conservando el signo) en vez de dejar al motor vibrando sin girar.
        if (pwm != 0 && abs(pwm) < GIRO_PWM_MIN_EFECTIVO) {
            pwm = (pwm > 0) ? GIRO_PWM_MIN_EFECTIVO : -GIRO_PWM_MIN_EFECTIVO;
        }

        // objetivoDeg > 0 (derecha): rueda izq adelante, rueda der atrás
        // objetivoDeg < 0 (izquierda): rueda izq atrás, rueda der adelante
        // Al aplicar el mismo PWM a un motor y su negativo al otro, ambas
        // ruedas giran en sentidos opuestos, logrando que el carro rote sobre
        // su propio eje en vez de desplazarse hacia adelante o atrás.
        motorIzquierdo(pwm);
        motorDerecho(-pwm);

        // ── Condición de llegada: dentro de tolerancia y estable ──────────────
        // No basta con que el error caiga dentro de la tolerancia una sola
        // vez (podría ser un cruce momentáneo por inercia); se exige que
        // permanezca dentro de la tolerancia durante GIRO_ESTABLE_MS
        // consecutivos antes de confirmar que el giro terminó.
        if (fabs(error) <= GIRO_TOLERANCIA_DEG) {
            if (!dentroDeTolerancia) {
                dentroDeTolerancia = true;
                tEstable = ahora;
            } else if (ahora - tEstable >= GIRO_ESTABLE_MS) {
                Serial.printf("[GIRO] Llegada confirmada. Yaw final: %.2f deg\n", yaw);
                break;
            }
        } else {
            // Si en algún momento el error vuelve a salirse de tolerancia,
            // se reinicia el conteo de estabilidad.
            dentroDeTolerancia = false;
        }

        delay(5);
    }

    detenerTodo();
    delay(150);   // pequeña pausa para que el carro asiente antes de seguir
}

// ── Macros de movimiento simple (avance/retroceso por tiempo) ────────────────
// Atajos de una línea para los tramos rectos de la secuencia de evasión, que
// se controlan por tiempo fijo (no por PID, a diferencia de los giros).
#define MOVIMIENTO(izq, der) motorIzquierdo(izq); motorDerecho(der)
#define PAUSA(ms)            delay(ms)

// ─────────────────────────────────────────────────────────────────────────────
// Secuencia de evasión (bloqueante)
//   retrocede → gira derecha 90° (PID) → avanza un poco →
//   gira izquierda 90° (PID) → avanza un poco → fin
// ─────────────────────────────────────────────────────────────────────────────

// Ejecuta la maniobra completa de evasión de obstáculo. Es bloqueante: el
// loop principal queda "congelado" dentro de esta función hasta que termina
// toda la secuencia, ya que cada paso depende del anterior (no se puede girar
// mientras se está retrocediendo, por ejemplo). Por eso se usa la bandera
// enEvasion para impedir que, mientras tanto, lleguen paquetes ESP-NOW o se
// dispare otra evasión que interfieran con los motores.
void secuenciaEvasion() {
    Serial.println("[EVASION] Iniciando secuencia...");
    enEvasion = true;

    // 1) Retrocede
    MOVIMIENTO(EVASION_RETROCESO_PWM, EVASION_RETROCESO_PWM);
    PAUSA(EVASION_RETROCESO_MS);
    detenerTodo();
    PAUSA(150);

    // 2) Gira a la derecha 90° (PID angular con MPU6050)
    girarPID(GIRO_ANGULO_DER);

    // 3) Avanza un poco
    MOVIMIENTO(EVASION_AVANCE_PWM, EVASION_AVANCE_PWM);
    PAUSA(EVASION_AVANCE_MS);
    detenerTodo();
    PAUSA(150);

    // 4) Gira a la izquierda 90° (PID angular con MPU6050)
    girarPID(GIRO_ANGULO_IZQ);

    // 5) Avanza un poco
    MOVIMIENTO(EVASION_AVANCE_PWM, EVASION_AVANCE_PWM);
    PAUSA(EVASION_AVANCE_MS);
    detenerTodo();

    enEvasion = false;

    // Reinicia el timeout para que el control retome sin falsa parada
    // Sin esto, "ultimoPaquete" seguiría teniendo el valor de antes de la
    // evasión (que pudo haber durado varios segundos), por lo que el loop
    // detectaría un timeout falso y cortaría los motores justo al recuperar
    // el control manual.
    ultimoPaquete = millis();
    Serial.println("[EVASION] Secuencia terminada. Retomando control por gestos.");
}

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW callback de recepción
// ─────────────────────────────────────────────────────────────────────────────

// Se ejecuta automáticamente cada vez que llega un paquete ESP-NOW válido.
// Aplica directamente las velocidades de PWM recibidas a los motores; no
// reinterpreta el gesto (cmd), confía en que el transmisor ya calculó los
// valores correctos de pwm_izq/pwm_der.
void onRecepcion(const uint8_t *mac, const uint8_t *data, int len) {
    if (enEvasion) return;   // ← la secuencia tiene prioridad total
    // Descarta paquetes que no tengan exactamente el tamaño esperado, como
    // protección básica contra datos corruptos o de otra estructura.
    if (len != sizeof(PaqueteControl)) return;

    PaqueteControl pkt;
    memcpy(&pkt, data, sizeof(PaqueteControl));

    ultimoPaquete = millis();
    motorActivo   = true;

    motorIzquierdo(pkt.pwm_izq);
    motorDerecho(pkt.pwm_der);

    Serial.printf("[CARRO] cmd:%d | izq:%4d  der:%4d\n",
                  pkt.cmd, pkt.pwm_izq, pkt.pwm_der);
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // ── Pines y LEDC ────────────────────────────────────────────────────────
    pinMode(PIN_IN1MR, OUTPUT);
    pinMode(PIN_IN1ML, OUTPUT);
    pinMode(PIN_IN2ML, OUTPUT);
    pinMode(PIN_IN2MR, OUTPUT);

    // Configura los 4 canales PWM (uno por entrada de cada motor) con la
    // frecuencia y resolución definidas arriba.
    ledcSetup(CH_RPWM_IZQ, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LPWM_IZQ, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RPWM_DER, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LPWM_DER, PWM_FREQ, PWM_RESOLUTION);

    // Asocia cada canal LEDC a su pin físico correspondiente.
    ledcAttachPin(PIN_RPWM_IZQ, CH_RPWM_IZQ);   // GPIO15
    ledcAttachPin(PIN_LPWM_IZQ, CH_LPWM_IZQ);   // GPIO12
    ledcAttachPin(PIN_RPWM_DER, CH_RPWM_DER);   // GPIO13
    ledcAttachPin(PIN_LPWM_DER, CH_LPWM_DER);   // GPIO14

    // Motores apagados desde el arranque, antes de inicializar cualquier
    // sensor o comunicación, por seguridad.
    detenerTodo();

    // ── I2C compartido (VL53L0X + MPU6050) ─────────────────────────────────
    // Ambos sensores comparten el mismo bus I2C (direcciones distintas), por
    // lo que basta una sola llamada a Wire.begin() con los pines por defecto.
    Wire.begin();

    // ── VL53L0X ─────────────────────────────────────────────────────────────
    // A diferencia del MPU6050, aquí un fallo de inicialización solo se
    // reporta por serial pero no detiene el programa (no hay while(1)); el
    // carro seguiría funcionando con control manual, aunque sin evasión.
    if (!sensorDistancia.begin()) {
        Serial.println("[SENSOR] ERROR: VL53L0X no encontrado. Verifica el cableado.");
    } else {
        Serial.println("[SENSOR] VL53L0X listo.");
    }

    // ── MPU6050 ─────────────────────────────────────────────────────────────
    // Igual que con el VL53L0X, un fallo aquí no detiene el programa, pero sí
    // dejaría sin efecto los giros por PID (ya que calibrarGyroZ() y las
    // lecturas posteriores dependerían de un sensor no inicializado).
    if (!mpu.begin()) {
        Serial.println("[MPU] ERROR: MPU6050 no encontrado. Verifica el cableado.");
    } else {
        // Rango amplio de giroscopio para no saturar la lectura durante
        // giros rápidos del carro.
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        // Filtro pasa-bajas para reducir el ruido de alta frecuencia antes
        // de integrar la velocidad angular.
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("[MPU] MPU6050 listo.");
        // Calibración del offset del giroscopio; el carro debe permanecer
        // quieto durante este paso para que la medición sea válida.
        calibrarGyroZ();
    }

    // ── ESP-NOW ─────────────────────────────────────────────────────────────
    // Mismo canal WiFi (6) que usa el control de gestos: ambos extremos deben
    // coincidir en el canal para poder comunicarse por ESP-NOW.
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    esp_now_init();
    esp_now_register_recv_cb(onRecepcion);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, macControl, 6);
    peer.channel = 6;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("[CARRO] Listo. Esperando paquetes del control...");
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    unsigned long ahora = millis();

    // ── Timeout de motores (solo si no hay evasión activa) ───────────────────
    // Si pasó más de TIMEOUT_MS desde el último paquete recibido (por ejemplo,
    // el control se apagó o se perdió la señal), se detienen los motores por
    // seguridad. Se omite esta verificación durante la evasión, ya que en ese
    // momento los motores son controlados directamente por secuenciaEvasion(),
    // no por paquetes ESP-NOW.
    if (!enEvasion && motorActivo && (ahora - ultimoPaquete > TIMEOUT_MS)) {
        detenerTodo();
        motorActivo = false;
        Serial.println("[CARRO] Timeout — motores detenidos.");
    }

    // ── Lectura del sensor de distancia (solo si no hay evasión activa) ──────
    // Se evita leer el sensor o disparar una nueva evasión mientras ya hay una
    // en curso, ya que secuenciaEvasion() es bloqueante y, en la práctica,
    // mientras se ejecuta este "if" ni siquiera se vuelve a evaluar hasta que
    // termine; el chequeo de enEvasion es una protección adicional de
    // claridad/seguridad ante futuras modificaciones del código.
    if (!enEvasion && (ahora - ultimaSensor >= SENSOR_INTERVALO_MS)) {
        ultimaSensor = ahora;

        VL53L0X_RangingMeasurementData_t medicion;
        sensorDistancia.rangingTest(&medicion, false);

        // RangeStatus == 4 indica una medición fuera de rango o no válida,
        // según la librería de Adafruit/ST; se descarta en ese caso.
        if (medicion.RangeStatus != 4) {
            uint16_t dist = medicion.RangeMilliMeter;
            Serial.printf("[SENSOR] Distancia: %4d mm\n", dist);

            // ── Disparar evasión si hay obstáculo a ≤ 8 cm ──────────────────
            if (dist <= DISTANCIA_EVASION_MM) {
                secuenciaEvasion();   // bloqueante; retorna cuando termina
            }
        } else {
            Serial.println("[SENSOR] Distancia: -- (fuera de rango)");
        }
    }

    delay(10);
}
