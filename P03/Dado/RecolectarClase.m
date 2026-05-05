% Práctica 03. Acelerómetro y Giroscopio:
% Recolección de Datos por Cara del Dado desde ESP32
%
% - Christian Emmanuel Castruita Alaniz
% - Del Hoyo Gómez Karla Stephanie
% - Pablo David Sánchez García
%
% Descripción:
% Este script de MATLAB recolecta muestras de Pitch y Roll enviadas por el
% ESP32 a través del puerto serie para una cara específica del dado. El usuario
% indica el número de cara (1-6) antes de iniciar la captura. Durante 10 segundos
% el script lee continuamente el puerto serie, normaliza cada par de valores al
% rango [-1, 1] y los almacena en memoria. Al finalizar, los datos se guardan
% automáticamente en un archivo ClaseN.mat con el nombre de variable correspondiente
% a la cara recolectada. Se genera además una gráfica de verificación con la
% evolución temporal de los ángulos normalizados de Pitch y Roll.
% Repetir este proceso una vez por cada cara del dado (6 ejecuciones en total).
% Requiere: ESP32 transmitiendo "pitch,roll" en formato CSV por puerto serie.

% Limpia el espacio de trabajo, la consola y cierra figuras previas
clear; clc; close all;

% Solicita al usuario la cara del dado que se va a recolectar
numCara = input('Numero de cara del dado a recolectar (1-6): ');

% Valida que el número ingresado corresponda a una cara válida del dado
if numCara < 1 || numCara > 6
    error('Debe ser un entero entre 1 y 6');
end

% --- Parámetros de temporización y normalización ---
ts = 0.1;       % Período de muestreo en segundos (~10 Hz, igual que el ESP32)
t  = 0:0.1:10;  % Vector de tiempo: 101 muestras en 10 segundos
maxValue = 1023; % Valor máximo del rango discretizado (10 bits)
minValue = 0;    % Valor mínimo del rango discretizado

% Reserva memoria para los datos crudos y normalizados
datos  = zeros(length(t), 2); % Valores enteros recibidos por serie (pitch, roll)
datosN = zeros(length(t), 2); % Valores normalizados al rango [-1, 1]

% Abre la conexión serie con el ESP32
% Ajustar el puerto COM según el dispositivo y sistema operativo
arduino = serialport("COM8", 115200);

% Pausa para permitir que el ESP32 complete su inicialización
pause(4);

% Indica al usuario la posición requerida del dado antes de iniciar
fprintf('Sostenga el dado con la CARA %d hacia arriba.\n', numCara);
fprintf('Recolectando datos durante %d segundos. Mueva levemente el dado...\n', t(end));
pause(2);

% --- Bucle principal de recolección ---
for k = 1:length(t)
    tic; % Inicia el cronómetro para controlar el período de muestreo

    % Descarta datos residuales en el buffer para leer siempre la muestra más reciente
    flush(arduino);

    % Lee una línea del puerto serie y elimina espacios en blanco
    data = strtrim(readline(arduino));

    % Separa los dos valores enviados por el ESP32 (pitch, roll) separados por coma
    valores = str2double(split(data, ","));

    % Almacena solo si se recibieron exactamente dos valores válidos
    if length(valores) == 2
        datos(k, :)  = valores;

        % Normaliza al rango [-1, 1] requerido por la red neuronal
        % pN = 2 * (x - min) / (max - min) - 1
        datosN(k, :) = 2 * (valores' - minValue) ./ (maxValue - minValue) - 1;
    end

    % Espera activa hasta completar el período de muestreo de 100 ms
    while toc < ts; end
end

% Cierra la conexión serie al terminar la recolección
clear arduino;
disp('Datos recolectados.');

% Extrae columnas normalizadas para guardar y graficar
valueN1 = datosN(:, 1); % Pitch normalizado
valueN2 = datosN(:, 2); % Roll normalizado

% Ensambla la matriz de entradas de la clase (formato: 2 x N muestras)
% y guarda en el archivo correspondiente a la cara recolectada
P_temp = [valueN1'; valueN2'];
switch numCara
    case 1, P1 = P_temp; save('Clase1.mat', 'P1');
    case 2, P2 = P_temp; save('Clase2.mat', 'P2');
    case 3, P3 = P_temp; save('Clase3.mat', 'P3');
    case 4, P4 = P_temp; save('Clase4.mat', 'P4');
    case 5, P5 = P_temp; save('Clase5.mat', 'P5');
    case 6, P6 = P_temp; save('Clase6.mat', 'P6');
end

fprintf('Guardado: Clase%d.mat\n', numCara);

% --- Visualización rápida para verificar la calidad de los datos recolectados ---
figure
subplot(2, 1, 1);
plot(t, valueN1); grid on;
title(sprintf('Cara %d - pitch normalizado', numCara));

subplot(2, 1, 2);
plot(t, valueN2); grid on;
title(sprintf('Cara %d - roll normalizado', numCara));
xlabel('tiempo [s]');