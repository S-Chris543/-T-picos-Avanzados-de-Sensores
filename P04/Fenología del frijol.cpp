/*
  Práctica 04. Sensor de presión BMP180:
  Monitor de Fenología del Frijol por Unidades Calor (UC)

  - Christian Emmanuel Castruita Alaniz
  - Del Hoyo Gómez Karla Stephanie
  - Pablo David Sánchez García

  Descripción:
  Este programa para ESP32 utiliza el sensor BMP180 para monitorear el desarrollo
  fenológico del frijol variedad "Rojo de Seda". Cada 2 segundos registra la temperatura
  ambiente y lleva el seguimiento de la temperatura máxima y mínima del día. Al completar
  cada día, calcula las Unidades Calor (UC) acumuladas usando la fórmula de temperatura
  base (TB = 10 °C), y determina en qué etapa fenológica se encuentra el cultivo
  (Emergencia → Cosecha completa). Los resultados se muestran en el monitor serial como
  una tabla visual con barra de progreso y resaltado de la etapa activa.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Objeto para manejar el sensor BMP180
Adafruit_BMP085 bmp;

// Temperatura base para el cálculo de Unidades Calor (°C)
// Por debajo de esta temperatura el frijol no acumula desarrollo
const float TB = 10.0;

// UC totales necesarias para alcanzar cosecha completa (límite de la barra de progreso)
const float UC_MAX = 153.24;

// Acumulado total de Unidades Calor desde el inicio del ciclo
float UC_acumuladas = 0.0;

// Temperaturas extremas del día actual (se reinician cada día)
float tempMax = -999.0;   // Inicia muy bajo para que cualquier lectura lo supere
float tempMin =  999.0;   // Inicia muy alto para que cualquier lectura lo supere

// Contador de días transcurridos desde el inicio del ciclo
int dia = 0;

// Período de un día completo en milisegundos (86400000 ms = 24 horas)
// Puede reducirse para pruebas rápidas sin esperar un día real
const unsigned long PERIODO_DIA     = 86400000UL;

// Intervalo de muestreo de temperatura (cada 2 segundos)
const unsigned long PERIODO_MUESTRA = 2000;

// Marcas de tiempo para controlar los intervalos sin usar delay()
unsigned long ultimaMuestra = 0;
unsigned long inicioDia     = 0;

// Estructura que define cada etapa fenológica del frijol
struct Etapa
{
  const char* nombre;   // Nombre de la etapa
  float ucMin;          // UC mínimas para entrar a esta etapa
  float ucMax;          // UC máximas antes de pasar a la siguiente
};

// Tabla de etapas fenológicas del frijol variedad Rojo de Seda
// Basada en umbrales de Unidades Calor acumuladas
const Etapa etapas[] = {
  { "Emergencia",         0.00,   12.89 },
  { "Formacion de guias", 12.89,  64.41 },
  { "Floracion",          64.41,  91.33 },
  { "Formacion de vaina", 91.33,  99.31 },
  { "Llenado de vainas",  99.31, 112.77 },
  { "Maduracion",        112.77, 153.24 },
  { "Cosecha completa",  153.24, 9999.0 },  // 9999 = sin límite superior
};
const int N_ETAPAS = 7;

/*
  Retorna el índice de la etapa fenológica correspondiente
  a las UC acumuladas recibidas.
  Si ninguna coincide exactamente, devuelve la última etapa.
*/
int obtenerEtapaIdx(float uc)
{
  for (int i = 0; i < N_ETAPAS; i++)
    if (uc >= etapas[i].ucMin && uc < etapas[i].ucMax) return i;
  return N_ETAPAS - 1;
}

/*
  Dibuja una barra de progreso ASCII en el monitor serial.
  La barra tiene 30 caracteres de ancho y representa el porcentaje
  de UC acumuladas respecto al total necesario para la cosecha.
*/
void imprimirBarra(float uc)
{
  int filled = (int)min(30.0f, (uc / UC_MAX) * 30.0f);
  int pct    = (int)min(100.0f, (uc / UC_MAX) * 100.0f);
  Serial.print("  [");
  for (int i = 0; i < 30; i++) Serial.print(i < filled ? '=' : '-');
  Serial.printf("] %3d%%\n", pct);
}

/*
  Redibuja la pantalla completa en el monitor serial.
  Muestra: encabezado, temperatura del día, UC acumuladas,
  barra de progreso y tabla de etapas fenológicas con
  resaltado de la etapa activa y marcado de las completadas.
*/
void imprimirPantalla(float uc)
{
  int activa = obtenerEtapaIdx(uc);

  // Calcula las UC del día con la fórmula de temperatura base
  // UC = ((TMax + TMin) / 2) - TB  →  si es negativo se toma como 0
  float UC_dia_last = ((tempMax + tempMin) / 2.0) - TB;
  if (UC_dia_last < 0) UC_dia_last = 0;

  // Limpia el terminal desplazando con saltos de línea (compatible con Serial Monitor)
  for (int i = 0; i < 40; i++) Serial.println();

  // --- Encabezado ---
  Serial.println("  +===============================================+");
  Serial.println("  |     MONITOR DE FENOLOGIA DEL FRIJOL          |");
  Serial.println("  |         Variedad: Rojo de Seda               |");
  Serial.println("  +===============================================+");

  // --- Resumen del día actual ---
  Serial.printf("  | Dia: %-3d   Temp Max: %5.1f C  Min: %5.1f C  |\n",
                dia, tempMax, tempMin);
  Serial.printf("  | UC del dia: %6.2f     UC Acumuladas: %6.2f  |\n",
                UC_dia_last, uc);

  // --- Barra de progreso hacia cosecha ---
  Serial.println("  +===============================================+");
  Serial.println("  | Progreso hacia cosecha:                       |");
  imprimirBarra(uc);

  // --- Tabla de etapas fenológicas ---
  Serial.println("  +-----------------------------------------------+");
  Serial.println("  |  #  | ETAPA FENOLOGICA    |  UC MIN |  UC MAX |");
  Serial.println("  +-----+--------------------+---------+---------+");

  for (int i = 0; i < N_ETAPAS; i++)
  {
    bool es_activa     = (i == activa);
    bool es_completada = (i <  activa);

    // Ícono de estado: OK = completada, >> = activa, vacío = pendiente
    const char* icono;
    if      (es_completada) icono = " OK ";
    else if (es_activa)     icono = ">>  ";
    else                    icono = "    ";

    // El último umbral se muestra como "---" porque no tiene límite superior real
    char ucMaxStr[8];
    if (etapas[i].ucMax >= 9999) snprintf(ucMaxStr, sizeof(ucMaxStr), "  ---  ");
    else                         snprintf(ucMaxStr, sizeof(ucMaxStr), "%7.2f", etapas[i].ucMax);

    Serial.printf("  | %s| %-19s|%7.2f  |%s |\n",
                  icono,
                  etapas[i].nombre,
                  etapas[i].ucMin,
                  ucMaxStr);

    // Resalta la etapa activa con una sección especial debajo de su fila
    if (es_activa)
    {
      Serial.println("  +-----+--------------------+---------+---------+");
      Serial.printf( "  |  >> ETAPA ACTUAL: %-28s |\n", etapas[i].nombre);
      Serial.println("  +-----+--------------------+---------+---------+");
    }
  }

  // --- Pie de pantalla con info del sensor ---
  Serial.println("  +===============================================+");
  Serial.printf( "  | Sensor BMP180  Temp actual: -- (cada 2s)      |\n");
  Serial.println("  +===============================================+");
}

void setup()
{
  Serial.begin(115200);

  // Inicializa el BMP180 en modo ultra alta resolución
  if (!bmp.begin(BMP085_ULTRAHIGHRES))
  {
    Serial.println("BMP180 no encontrado. Verifica la conexion.");
    while (1); // Bloqueo intencional: revisar cableado I²C y alimentación
  }

  // Registra el instante de inicio para controlar los intervalos
  inicioDia     = millis();
  ultimaMuestra = millis();

  // Muestra la pantalla inicial con UC = 0 (inicio del ciclo)
  imprimirPantalla(0);
}

void loop()
{
  unsigned long ahora = millis();

  // --- Muestreo cada 2 segundos ---
  // Usa diferencia de tiempo para no bloquear el programa con delay()
  if (ahora - ultimaMuestra >= PERIODO_MUESTRA)
  {
    ultimaMuestra = ahora;

    float t = bmp.readTemperature();

    // Actualiza los extremos del día con la lectura actual
    if (t > tempMax) tempMax = t;
    if (t < tempMin) tempMin = t;

    // Muestra una línea compacta en tiempo real sin redibujar la tabla completa
    // \r sobreescribe la misma línea en el terminal
    Serial.printf("\r  [BMP180] Temp: %5.2f C  |  Max: %5.2f  Min: %5.2f      ",
                  t, tempMax, tempMin);
  }

  // --- Cierre de día: calcular UC y redibujar pantalla ---
  if (ahora - inicioDia >= PERIODO_DIA)
  {
    inicioDia = ahora;
    dia++;

    // Fórmula de Unidades Calor diarias
    // UC = ((TMax + TMin) / 2) - TB
    float UC_dia = ((tempMax + tempMin) / 2.0) - TB;
    if (UC_dia < 0) UC_dia = 0;  // No se acumula desarrollo por debajo de TB

    UC_acumuladas += UC_dia;

    // Redibuja la pantalla completa con los datos actualizados del nuevo día
    imprimirPantalla(UC_acumuladas);

    // Reinicia los extremos de temperatura para el nuevo día
    tempMax = -999.0;
    tempMin =  999.0;
  }
}
