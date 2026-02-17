/*
Práctica 01. Sistema de Posicionamiento Global GPS: GEOCERCA

  - Christian Emmanuel Castruita Alaniz: Desarrollo principal del código GPS y geocerca.
  - Del Hoyo Gómez Karla Stephanie: Pruebas de funcionamiento y documentación.
  - Pablo David Sánchez García: Apoyo en cálculos y optimización de la geocerca.

  Descripción:
  Este código implementa una geocerca circular con radio de 50 metros centrada
  en coordenadas definidas. Utiliza un módulo GPS NEO-6M conectado a un ESP32 
  para obtener latitud, longitud, altitud, velocidad y número de satélites.
  Cada segundo calcula la distancia al centro de la geocerca usando la fórmula
  de Haversine y muestra si el dispositivo está dentro o fuera del área definida.
*/

#include <Arduino.h>      
#include <TinyGPSPlus.h>   
#include <HardwareSerial.h> 
#include <math.h>         

TinyGPSPlus gps;           
HardwareSerial GPSSerial(1); // Puerto serial hardware para comunicación con GPS

#define GPS_RX 16
#define GPS_TX 17

// Coordenadas del centro de la geocerca y radio en metros
#define GEO_LAT 22.784083
#define GEO_LNG -102.616722
#define GEO_RADIUS 50  // radio de la geocerca en metros

void setup() {
  Serial.begin(115200); 
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // Configuración del GPS
  Serial.println("GPS NEO-6M iniciado...");
}

// Función para calcular la distancia entre dos coordenadas usando Haversine
double calcularDistancia(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Radio de la Tierra en metros
  double latRad1 = lat1 * DEG_TO_RAD; // Convierte grados a radianes
  double latRad2 = lat2 * DEG_TO_RAD;
  double deltaLat = (lat2 - lat1) * DEG_TO_RAD;
  double deltaLon = (lon2 - lon1) * DEG_TO_RAD;

  // Fórmula de Haversine
  double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
             cos(latRad1) * cos(latRad2) *
             sin(deltaLon / 2) * sin(deltaLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distancia = R * c;
  return distancia; // Devuelve distancia en metros
}

void loop() {

  // Leer datos disponibles del GPS
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  // Si hay una nueva posición disponible
  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    double distancia = calcularDistancia(lat, lng, GEO_LAT, GEO_LNG);

    Serial.println("------ DATOS GPS ------");
    Serial.print("Latitud: ");
    Serial.println(lat, 6);

    Serial.print("Longitud: ");
    Serial.println(lng, 6);

    Serial.print("Velocidad (km/h): ");
    Serial.println(gps.speed.kmph());

    Serial.print("Altitud (m): ");
    Serial.println(gps.altitude.meters());

    Serial.print("Satelites: ");
    Serial.println(gps.satellites.value());

    Serial.print("Distancia al centro de la geocerca (m): ");
    Serial.println(distancia, 2);

    // Verifica si el dispositivo está dentro o fuera de la geocerca
    if (distancia <= GEO_RADIUS) {
      Serial.println("Estado: DENTRO de la geocerca");
    } else {
      Serial.println("Estado: FUERA de la geocerca");
    }

    Serial.println("------------------------\n");
    delay(1000); // Frecuencia de muestreo de 1 Hz
  }
}
