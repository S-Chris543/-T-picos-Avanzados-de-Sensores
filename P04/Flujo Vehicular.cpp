/*
  Práctica 04. Sensor de presión BMP180:
  Sistema de detección y clasificación de flujo vehicular

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor BMP180 para detectar y clasificar vehículos
  en tránsito mediante los cambios de presión atmosférica que generan al pasar. Mantiene
  una línea base dinámica de presión y compara cada lectura contra ella; si el delta supera
  un umbral, registra el inicio de un pulso. Al finalizar el pulso aplica filtros de duración
  (histéresis) para descartar ruido, luego clasifica el vehículo en Moto/Bici, Auto/Camioneta
  o Camión/Bus según la magnitud del pico de presión, y estima su velocidad por la duración
  del pulso. Cada 10 vehículos válidos imprime un resumen estadístico en el monitor serial.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Objeto para manejar el sensor BMP180
Adafruit_BMP085 bmp;

// ─── Umbrales de detección ────────────────────────────────────────────────────

// Delta de presión mínimo para considerar que un vehículo está pasando (Pa)
// Subir este valor si hay falsos positivos por viento o ruido ambiental
const float UMBRAL_PA = 8.0;

// Delta de presión por debajo del cual se considera que el vehículo ya pasó (Pa)
// Menor que UMBRAL_PA → crea histéresis para evitar detecciones múltiples por un mismo vehículo
const float UMBRAL_FIN_PA = 4.0;

// Número de muestras usadas para calcular la media móvil y la línea base
const int VENTANA_MEDIA = 20;

// Tiempo mínimo entre dos detecciones consecutivas (ms)
// Evita que un mismo vehículo se cuente varias veces por oscilaciones del sensor
const unsigned long DEBOUNCE = 1500;

// Duración mínima de un pulso válido (ms); pulsos más cortos se descartan como ruido puntual
const unsigned long MIN_PULSO = 80;

// Duración máxima de un pulso válido (ms); pulsos más largos se descartan como ruido sostenido
const unsigned long MAX_PULSO = 800;

// ─── Umbrales de clasificación vehicular ─────────────────────────────────────

// Delta de presión máximo esperado para motos y bicicletas (Pa)
// Generan poco desplazamiento de aire por su menor masa frontal
const float PA_MOTO = 12.0;

// Delta de presión máximo esperado para autos y camionetas (Pa)
// Valores superiores a este umbral corresponden a camiones o autobuses
const float PA_AUTO = 28.0;

// ─── Buffer circular para media móvil ────────────────────────────────────────

// Almacena las últimas VENTANA_MEDIA lecturas de presión
float buffer[20] = {0};

// Índice actual de escritura en el buffer circular
int bufIdx = 0;

// ─── Estado de detección ─────────────────────────────────────────────────────

// Indica si actualmente se está registrando el paso de un vehículo
bool vehiculoActivo = false;

// Pico máximo de delta de presión registrado durante el pulso actual
float maxDeltaP = 0;

// Presión de referencia sin vehículos; se actualiza lentamente en reposo
float lineaBase = 0;

// Instante en que comenzó el pulso actual (ms desde arranque)
unsigned long tiempoInicio = 0;

// Instante de la última detección válida, usado para el debounce
unsigned long ultimaDeteccion = 0;

// ─── Contadores de vehículos ──────────────────────────────────────────────────
int contMotos      = 0;   // Motos y bicicletas detectadas
int contAutos      = 0;   // Autos y camionetas detectadas
int contCamiones   = 0;   // Camiones y autobuses detectados
int contTotal      = 0;   // Total de vehículos válidos en el bloque actual
int contRechazados = 0;   // Pulsos descartados por duración fuera de rango

// Longitud promedio asumida de un vehículo (m), usada para estimar velocidad
// v = largo / tiempo_pulso  →  velocidad en m/s, luego convertida a km/h
const float LARGO_PROMEDIO_M = 4.5;

/*
  Retorna una cadena con la categoría del vehículo según el pico de presión.
  Moto/Bici < PA_MOTO ≤ Auto/Camioneta < PA_AUTO ≤ Camión/Bus
*/
String clasificarVehiculo(float dp)
{
  if (dp < PA_MOTO) return "MOTO/BICI  ";
  if (dp < PA_AUTO) return "AUTO/CAMION";
  return                   "CAMION/BUS ";
}

/*
  Limpia el terminal e imprime el encabezado del sistema con los parámetros
  de configuración activos y el encabezado de la tabla de detecciones.
*/
void imprimirEncabezado()
{
  for (int i = 0; i < 30; i++) Serial.println();
  Serial.println("  +================================================+");
  Serial.println("  |     SISTEMA DE FLUJO VEHICULAR - BMP180        |");
  Serial.println("  +================================================+");
  Serial.printf( "  | Umbral deteccion  : %5.1f Pa                   |\n", UMBRAL_PA);
  Serial.printf( "  | Umbral fin pulso  : %5.1f Pa (histeresis)      |\n", UMBRAL_FIN_PA);
  Serial.printf( "  | Ventana base      : %5d muestras              |\n", VENTANA_MEDIA);
  Serial.printf( "  | Debounce          : %5lu ms                    |\n", DEBOUNCE);
  Serial.printf( "  | Duracion valida   : %3lu – %3lu ms              |\n", MIN_PULSO, MAX_PULSO);
  Serial.println("  +================================================+");
  Serial.println("  | TIPO        | DELTA P | DURACION | VELOCIDAD   |");
  Serial.println("  +-------------+---------+----------+-------------+");
}

void setup()
{
  Serial.begin(115200);

  // Inicializa el BMP180 en modo ultra alta resolución
  if (!bmp.begin(BMP085_ULTRAHIGHRES))
  {
    Serial.println("BMP180 no encontrado.");
    while (1); // Bloqueo intencional: revisar cableado I²C y alimentación
  }

  // Calibración inicial: llena el buffer con VENTANA_MEDIA lecturas reales
  // para establecer una línea base estable antes de comenzar la detección
  Serial.println("  Calibrando linea base... espera 3 segundos.");
  for (int i = 0; i < VENTANA_MEDIA; i++)
  {
    buffer[i] = bmp.readPressure();
    delay(150);  // 150 ms × 20 muestras ≈ 3 segundos de calibración
  }

  // Promedia las muestras iniciales para fijar la línea base de referencia
  float suma = 0;
  for (int i = 0; i < VENTANA_MEDIA; i++) suma += buffer[i];
  lineaBase = suma / VENTANA_MEDIA;

  imprimirEncabezado();
}

void loop()
{
  unsigned long ahora = millis();

  // 1. Leer presión absoluta actual del sensor
  float presActual = bmp.readPressure();

  // 2. Insertar lectura en el buffer circular (sobrescribe la muestra más antigua)
  buffer[bufIdx] = presActual;
  bufIdx = (bufIdx + 1) % VENTANA_MEDIA;

  // 3. Calcular la media móvil de las últimas VENTANA_MEDIA lecturas
  float suma = 0;
  for (int i = 0; i < VENTANA_MEDIA; i++) suma += buffer[i];
  float media = suma / VENTANA_MEDIA;

  // 4. Calcular el delta absoluto respecto a la línea base sin vehículos
  float deltaP = abs(presActual - lineaBase);

  // 5. Detectar INICIO de pulso vehicular
  // Condiciones: sin vehículo activo + delta supera umbral + debounce cumplido
  if (!vehiculoActivo
      && deltaP > UMBRAL_PA
      && (ahora - ultimaDeteccion) > DEBOUNCE)
  {
    vehiculoActivo = true;
    tiempoInicio   = ahora;
    maxDeltaP      = deltaP;
  }

  // 6. Durante el pulso: rastrear el pico máximo de presión
  if (vehiculoActivo)
  {
    if (deltaP > maxDeltaP) maxDeltaP = deltaP;

    unsigned long duracionActual = ahora - tiempoInicio;

    // Abortar si el pulso excede MAX_PULSO mientras el delta sigue alto
    // Indica ruido sostenido (viento fuerte, vibración) en lugar de un vehículo real
    if (duracionActual > MAX_PULSO && deltaP > UMBRAL_FIN_PA)
    {
      vehiculoActivo  = false;
      ultimaDeteccion = ahora;
      contRechazados++;
      Serial.printf("  | [RECHAZADO - pulso demasiado largo: %lu ms]    |\n", duracionActual);

      // Resetea la línea base con la media actual para adaptarse al nuevo entorno
      lineaBase = media;
      return;
    }

    // 7. Detectar FIN del pulso: delta cae por debajo del umbral de histéresis
    if (deltaP < UMBRAL_FIN_PA)
    {
      vehiculoActivo  = false;
      ultimaDeteccion = ahora;
      unsigned long duracionMs = ahora - tiempoInicio;

      // Descartar pulsos demasiado cortos (ruido puntual, insecto, vibración breve)
      if (duracionMs < MIN_PULSO)
      {
        contRechazados++;
        Serial.printf("  | [RECHAZADO - pulso muy corto: %lu ms]          |\n", duracionMs);
        return;
      }

      // ── Vehículo válido: clasificar, calcular velocidad y registrar ──

      String tipo = clasificarVehiculo(maxDeltaP);

      // Estimación de velocidad: v = largo / tiempo  (m/s → km/h)
      float velocidadKmh = (LARGO_PROMEDIO_M / (duracionMs / 1000.0)) * 3.6;

      contTotal++;

      // Incrementa el contador de la categoría correspondiente
      if      (maxDeltaP < PA_MOTO) contMotos++;
      else if (maxDeltaP < PA_AUTO) contAutos++;
      else                          contCamiones++;

      Serial.printf("  | %s | %5.1f Pa | %6lu ms | %6.1f km/h  |\n",
                    tipo.c_str(), maxDeltaP, duracionMs, velocidadKmh);

      // Actualiza la línea base con la media actual (solo en reposo post-pulso)
      lineaBase = media;
    }
  }
  else
  {
    // Sin vehículo activo: actualiza la línea base muy lentamente (factor 0.005)
    // para adaptarse a cambios graduales de presión ambiental sin perder sensibilidad
    lineaBase = lineaBase * 0.995 + media * 0.005;
  }

  // 8. Imprimir resumen estadístico cada 10 vehículos válidos y reiniciar contadores
  if (contTotal > 0 && contTotal % 10 == 0)
  {
    Serial.println("  +================================================+");
    Serial.println("  |                 RESUMEN PARCIAL                |");
    Serial.println("  +================================================+");
    Serial.printf( "  | Total validos    : %4d veh.                   |\n", contTotal);
    Serial.printf( "  | Rechazados       : %4d (ruido/viento)         |\n", contRechazados);
    Serial.printf( "  | Motos/Bicis      : %4d  (%3.0f%%)               |\n",
                   contMotos,    (contMotos    * 100.0) / contTotal);
    Serial.printf( "  | Autos/Camionetas : %4d  (%3.0f%%)               |\n",
                   contAutos,    (contAutos    * 100.0) / contTotal);
    Serial.printf( "  | Camiones/Buses   : %4d  (%3.0f%%)               |\n",
                   contCamiones, (contCamiones * 100.0) / contTotal);
    Serial.println("  +================================================+");
    Serial.println("  | TIPO        | DELTA P | DURACION | VELOCIDAD   |");
    Serial.println("  +-------------+---------+----------+-------------+");

    // Reinicia el conteo para el siguiente bloque de 10 vehículos
    contTotal = 0;
  }
}
