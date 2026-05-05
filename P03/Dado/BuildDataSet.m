% Práctica 03. Acelerómetro y Giroscopio:
% Construcción del Conjunto de Entrenamiento para la Red Neuronal
%
% - Christian Emmanuel Castruita Alaniz
% - Del Hoyo Gómez Karla Stephanie
% - Pablo David Sánchez García
%
% Descripción:
% Este script ensambla el conjunto de entrenamiento completo a partir de los
% seis archivos de datos recolectados con RecolectarClase.m (Clase1.mat a
% Clase6.mat). Concatena horizontalmente las matrices de entradas de cada
% cara para formar la matriz P (2 x Q muestras totales). Paralelamente,
% construye la matriz de salidas deseadas T (6 x Q) usando codificación
% one-hot compatible con la función de activación tansig: la neurona
% correspondiente a la cara correcta toma el valor +1 y las cinco restantes
% toman el valor -1. El resultado se guarda en DataSet.mat para su uso
% inmediato en Train_Dado.m.
% Requiere: Clase1.mat, Clase2.mat, Clase3.mat, Clase4.mat, Clase5.mat,
%           Clase6.mat (generados con RecolectarClase.m)

% Limpia el espacio de trabajo y la consola antes de iniciar
clear; clc;

% Carga los datos recolectados de cada cara del dado
% Cada archivo contiene una matriz PN de tamaño 2 x N
% (fila 1: pitch normalizado, fila 2: roll normalizado)
load Clase1.mat  % -> P1
load Clase2.mat  % -> P2
load Clase3.mat  % -> P3
load Clase4.mat  % -> P4
load Clase5.mat  % -> P5
load Clase6.mat  % -> P6

% Concatena horizontalmente las seis clases para formar la matriz de entradas
% P tiene tamaño 2 x (N1 + N2 + ... + N6)
P = [P1 P2 P3 P4 P5 P6];

% Obtiene el número de muestras por clase para construir las etiquetas
N1 = size(P1, 2);  N2 = size(P2, 2);  N3 = size(P3, 2);
N4 = size(P4, 2);  N5 = size(P5, 2);  N6 = size(P6, 2);

% --- Codificación one-hot compatible con tansig ---
% Cara correcta = +1, resto de caras = -1
% Cada TN es una matriz 6 x NN donde NN es el número de muestras de esa cara
T1 = repmat([ 1;-1;-1;-1;-1;-1], 1, N1); % Solo la neurona 1 activa
T2 = repmat([-1; 1;-1;-1;-1;-1], 1, N2); % Solo la neurona 2 activa
T3 = repmat([-1;-1; 1;-1;-1;-1], 1, N3); % Solo la neurona 3 activa
T4 = repmat([-1;-1;-1; 1;-1;-1], 1, N4); % Solo la neurona 4 activa
T5 = repmat([-1;-1;-1;-1; 1;-1], 1, N5); % Solo la neurona 5 activa
T6 = repmat([-1;-1;-1;-1;-1; 1], 1, N6); % Solo la neurona 6 activa

% Concatena horizontalmente las etiquetas en el mismo orden que P
% T tiene tamaño 6 x (N1 + N2 + ... + N6)
T = [T1 T2 T3 T4 T5 T6];

% Guarda las matrices P y T en un único archivo para Train_Dado.m
save('DataSet.mat', 'P', 'T');

% Imprime un resumen del dataset generado para verificación
fprintf('DataSet.mat generado correctamente.\n');
fprintf('  Tamano P (entradas):  %s\n', mat2str(size(P)));
fprintf('  Tamano T (salidas):   %s\n', mat2str(size(T)));
fprintf('  Muestras por clase: %d %d %d %d %d %d\n', N1, N2, N3, N4, N5, N6);