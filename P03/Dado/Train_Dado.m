% Práctica 03. Acelerómetro y Giroscopio:
% Entrenamiento de Red Neuronal para Clasificación de Caras del Dado
%
% - Christian Emmanuel Castruita Alaniz
% - Del Hoyo Gómez Karla Stephanie
% - Pablo David Sánchez García
%
% Descripción:
% Este script entrena una red neuronal de dos capas (perceptrón multicapa)
% para clasificar la cara superior de un dado físico a partir de los ángulos
% de Pitch y Roll normalizados. El conjunto de entrenamiento se carga desde
% DataSet.mat, el cual contiene la matriz de entradas P (2 x Q muestras) y
% la matriz de salidas deseadas T (6 x Q muestras) en formato one-hot.
% La red se entrena con la función neuralTrain y su desempeño se evalúa
% calculando la exactitud sobre el mismo conjunto de entrenamiento. Al finalizar,
% los pesos, sesgos y parámetros de la red se guardan en Weights_Dado.mat
% para su uso posterior en PredecirEnVivo.m y en el ESP32.
% Requiere: DataSet.mat (generado con BuildDataSet.m) y las funciones
%           neuralTrain.m y neuralPredict.m

% Limpia el espacio de trabajo, la consola y cierra figuras previas
clc; clear; close all;

% Carga el conjunto de entrenamiento completo
% P: matriz de entradas  (2 x Q) — pitch y roll normalizados
% T: matriz de salidas   (6 x Q) — codificación one-hot por cara
load DataSet.mat

% --- Topología de la red neuronal ---
nodeHidden = 10;      % Número de neuronas en la capa oculta
                      % Rango recomendado: 8-12 para 6 clases
                      % Subir a 15 si la curva de error no converge

% Función de activación de la capa oculta: tansig mapea al rango (-1, 1)
fhidden = 'tansig';
% Función de activación de la capa de salida: tansig para hacer match
% con la codificación one-hot en [-1, 1] (−1 = inactivo, +1 = activo)
foutput = 'tansig';

%% Entrenamiento de la red neuronal
% neuralTrain devuelve los pesos W1, b1 (capa oculta) y W2, b2 (capa de salida)
% junto con el vector de error cuadrático medio por época
[W1, b1, W2, b2, emedio] = neuralTrain(P, T, nodeHidden, fhidden, foutput);

% Grafica la curva de aprendizaje para verificar la convergencia del entrenamiento
figure
plot(emedio); grid on;
xlabel('Epoca');
ylabel('Error cuadratico medio');
title('Curva de entrenamiento - 6 caras del dado');

%% Evaluación de exactitud sobre el conjunto de entrenamiento
Q        = size(P, 2); % Número total de muestras
aciertos = 0;

for q = 1:Q
    % Propagación hacia adelante muestra a muestra
    a1 = neuralPredict(W1, P(:, q), b1, fhidden); % Salida capa oculta
    a2 = neuralPredict(W2, a1,      b2, foutput);  % Salida capa de salida

    % La cara predicha y la real corresponden al índice de máxima activación
    [~, cara_pred] = max(a2);
    [~, cara_real] = max(T(:, q));

    if cara_pred == cara_real, aciertos = aciertos + 1; end
end

fprintf('Exactitud sobre entrenamiento: %.2f%% (%d/%d)\n', ...
        100 * aciertos / Q, aciertos, Q);

%% Guardado de pesos y parámetros de la red
% Weights_Dado.mat es consumido por PredecirEnVivo.m y por el firmware del ESP32
save('Weights_Dado.mat', 'W1', 'b1', 'W2', 'b2', 'fhidden', 'foutput', 'nodeHidden');
disp('Pesos guardados en Weights_Dado.mat');Sonnet 4.6AdaptativoClaude es IA y pue