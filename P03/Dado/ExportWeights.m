% Práctica 03. Acelerómetro y Giroscopio:
% Exportación de Pesos de la Red Neuronal a Cabecera C para ESP32
%
% - Christian Emmanuel Castruita Alaniz
% - Del Hoyo Gómez Karla Stephanie
% - Pablo David Sánchez García
%
% Descripción:
% Este script lee los pesos entrenados almacenados en Weights_Dado.mat y los
% convierte a un archivo de cabecera C (weights_chidos.h) que puede ser incluido
% directamente en el firmware del ESP32. Las matrices W1, b1, W2 y b2 se escriben
% como arreglos estáticos de tipo float, junto con las constantes de topología
% NN_INPUTS, NN_HIDDEN y NN_OUTPUTS y guardas de inclusión múltiple. De esta forma,
% la red neuronal entrenada en MATLAB queda embebida en el microcontrolador sin
% necesidad de comunicación externa durante la inferencia.
% Requiere: Weights_Dado.mat (generado con Train_Dado.m)
% Produce:  weights_chidos.h — copiar al directorio del sketch de Arduino IDE

% Limpia el espacio de trabajo y la consola antes de iniciar
clear; clc;

% Carga los pesos y parámetros de la red neuronal entrenada
% Variables esperadas: W1, b1, W2, b2, fhidden, foutput, nodeHidden
load Weights_Dado.mat

% --- Dimensiones de la red neuronal ---
% Se leen directamente de las matrices para evitar inconsistencias con nodeHidden
inputs  = size(W1, 2); % Número de entradas  (2: pitch y roll normalizados)
hidden  = size(W1, 1); % Número de neuronas en la capa oculta
outputs = size(W2, 1); % Número de salidas   (6: una por cara del dado)

fprintf('Exportando red %d-%d-%d a weights_chidos.h\n', inputs, hidden, outputs);

% Abre el archivo de cabecera para escritura en el directorio actual
fid = fopen('weights_chidos.h', 'w');

% --- Encabezado del archivo generado ---
% Incluye topología y funciones de activación como referencia para el desarrollador
fprintf(fid, '/*\n');
fprintf(fid, ' * weights_chidos.h  (generado automaticamente por ExportWeights.m)\n');
fprintf(fid, ' * Topologia: %d-%d-%d  | fhidden=%s  foutput=%s\n', ...
            inputs, hidden, outputs, fhidden, foutput);
fprintf(fid, ' */\n\n');

% Guardas de inclusión múltiple: evitan errores si el header se incluye más de una vez
fprintf(fid, '#ifndef WEIGHTS_H\n#define WEIGHTS_H\n\n');

% --- Constantes de topología ---
% Permiten que el código del ESP32 recorra los arreglos sin valores hardcodeados
fprintf(fid, '#define NN_INPUTS  %d\n', inputs);
fprintf(fid, '#define NN_HIDDEN  %d\n', hidden);
fprintf(fid, '#define NN_OUTPUTS %d\n\n', outputs);

% --- Pesos de la capa oculta W1 (NN_HIDDEN x NN_INPUTS) ---
% Cada fila es el vector de pesos de una neurona oculta
fprintf(fid, 'const float W1[NN_HIDDEN][NN_INPUTS] = {\n');
for i = 1:hidden
    fprintf(fid, '  {');
    for j = 1:inputs
        fprintf(fid, '%+.6ff', W1(i, j));
        if j < inputs, fprintf(fid, ', '); end
    end
    fprintf(fid, '}');
    if i < hidden, fprintf(fid, ','); end
    fprintf(fid, '\n');
end
fprintf(fid, '};\n\n');

% --- Sesgos de la capa oculta b1 (NN_HIDDEN x 1) ---
% Se insertan saltos de línea cada 5 valores para mejorar la legibilidad del header
fprintf(fid, 'const float b1[NN_HIDDEN] = {\n  ');
for i = 1:hidden
    fprintf(fid, '%+.6ff', b1(i));
    if i < hidden, fprintf(fid, ', '); end
    if mod(i, 5) == 0 && i < hidden, fprintf(fid, '\n  '); end
end
fprintf(fid, '\n};\n\n');

% --- Pesos de la capa de salida W2 (NN_OUTPUTS x NN_HIDDEN) ---
% Cada fila corresponde a una neurona de salida (una por cara del dado)
fprintf(fid, 'const float W2[NN_OUTPUTS][NN_HIDDEN] = {\n');
for i = 1:outputs
    fprintf(fid, '  {');
    for j = 1:hidden
        fprintf(fid, '%+.6ff', W2(i, j));
        if j < hidden, fprintf(fid, ', '); end
    end
    fprintf(fid, '}');
    if i < outputs, fprintf(fid, ','); end
    fprintf(fid, '\n');
end
fprintf(fid, '};\n\n');

% --- Sesgos de la capa de salida b2 (NN_OUTPUTS x 1) ---
fprintf(fid, 'const float b2[NN_OUTPUTS] = {\n  ');
for i = 1:outputs
    fprintf(fid, '%+.6ff', b2(i));
    if i < outputs, fprintf(fid, ', '); end
end
fprintf(fid, '\n};\n\n');

% Cierre de la guarda de inclusión múltiple
fprintf(fid, '#endif\n');

% Cierra el archivo
fclose(fid);

fprintf('Listo: weights_chidos.h generado en el directorio actual.\n');
fprintf('Copia ese archivo al folder del sketch Reconocimiento_gestos_2026.\n');