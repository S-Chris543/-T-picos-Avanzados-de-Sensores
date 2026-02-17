/*
Práctica 01. Sistema de Posicionamiento Global GPS:
Adquisición de datos GPS a 1 Hz durante 15 minutos
  
- Christian Emmanuel Castruita Alaniz: Desarrollo principal del código GPS y geocerca.
  - Del Hoyo Gómez Karla Stephanie: Pruebas de funcionamiento y documentación.
  - Pablo David Sánchez García: Apoyo en cálculos y optimización de la geocerca.


Descripción:
Este código lee los datos de latitud, longitud y altitud provenientes de un módulo GPS NEO-6M 
conectado a una ESP32. Los datos se almacenan en tres arreglos de 900 posiciones, equivalentes 
a 15 minutos de muestreo a una frecuencia de 1 Hz(1 muestra por segundo).

Una vez que se completan las 900 muestras, el programa imprime los arreglos en formato compatible 
con MATLAB para facilitar su análisis estadístico y graficación posterior.
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Puerto serial hardware para comunicación con GPS

#define GPS_RX 16
#define GPS_TX 17

// Configuración de muestreo
const int totalMuestras = 900;   // 15 minutos a 1 Hz
int indice = 0;                  // Índice para almacenar datos

// Arreglos para almacenar datos
double latitudes[totalMuestras];
double longitudes[totalMuestras];
double altitudes[totalMuestras];

// Variables para control de tiempo
unsigned long tiempoAnterior = 0;
const unsigned long intervalo = 1000;  // 1000 ms = 1 Hz

void setup() {

  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // Configuración del GPS
  Serial.println("GPS NEO-6M iniciado...");
}

void loop() {

  // Leer datos disponibles del GPS
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  // Si hay una nueva posición disponible
  if (gps.location.isValid() && gps.altitude.isValid()) {

    unsigned long tiempoActual = millis();

    // Controlar muestreo a 1 Hz sin usar delay
    if (tiempoActual - tiempoAnterior >= intervalo && indice < totalMuestras) {

      tiempoAnterior = tiempoActual;

      // Guardar datos en los arreglos
      latitudes[indice]  = gps.location.lat();
      longitudes[indice] = gps.location.lng();
      altitudes[indice]  = gps.altitude.meters();

      Serial.print("Muestra ");
      Serial.print(indice);
      Serial.println(" guardada");

      indice++;
    }

    // Cuando se completan las 900 muestras
    if (indice >= totalMuestras) {

      Serial.println("\nDatos para MATLAB");

      // Imprimir arreglo de latitud
      Serial.println("lat = [");
      for (int i = 0; i < totalMuestras; i++) {
        Serial.print(latitudes[i], 6);
        Serial.println(";");
      }
      Serial.println("];");

      // Imprimir arreglo de longitud
      Serial.println("lon = [");
      for (int i = 0; i < totalMuestras; i++) {
        Serial.print(longitudes[i], 6);
        Serial.println(";");
      }
      Serial.println("];");

      // Imprimir arreglo de altitud
      Serial.println("alt = [");
      for (int i = 0; i < totalMuestras; i++) {
        Serial.print(altitudes[i], 2);
        Serial.println(";");
      }
      Serial.println("];");

      Serial.println("Fin de captura");

      // Detener ejecución una vez finalizada la adquisición
      while (true);
    }
  }
}

