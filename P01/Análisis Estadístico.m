% Práctica 01. Sistema de Posicionamiento Global GPS:
% Análisis estadístico
%
% Christian Emmanuel Castruita Alaniz
% Del Hoyo Gómez Karla Stephanie
% Pablo David Sánchez García
%
% Descripción:
% El código de análisis estadístico procesa los datos de latitud, longitud y altitud obtenidos 
% durante la adquisición de señales GPS a 1 Hz durante 15 minutos, correspondientes a 900 muestras 
% por variable, con el objetivo de evaluar el comportamiento y la precisión del sistema de posicionamiento. 
% A partir de tres arreglos externos denominados latitud, longitud y altitud, el programa calcula para cada 
% uno el promedio, la desviación estándar y el error cuadrático medio (RMSE) respecto a la media, permitiendo 
% cuantificar tanto el valor central de las mediciones como su dispersión y estabilidad. Finalmente, 
% los resultados se muestran en un formato estructurado en la ventana de comandos de MATLAB, facilitando 
% la interpretación del desempeño del módulo GPS bajo distintas condiciones de medición y apoyando el análisis 
% comparativo y estadístico de los datos obtenidos.
clc;
clear;

%% Verificación de variables
if ~exist('latitud','var') || ~exist('longitud','var') || ~exist('altitud','var')
    error('Debes tener definidos los arreglos: latitud, longitud y altitud');
end

%% Cálculos estadísticos

% Latitud
lat_prom = mean(latitud);
lat_std  = std(latitud);
lat_rmse = sqrt(mean((latitud - lat_prom).^2));

% Longitud
lon_prom = mean(longitud);
lon_std  = std(longitud);
lon_rmse = sqrt(mean((longitud - lon_prom).^2));

% Altitud
alt_prom = mean(altitud);
alt_std  = std(altitud);
alt_rmse = sqrt(mean((altitud - alt_prom).^2));

%% Despliegue de resultados

fprintf('\nAnalisis estadistico\n\n');

fprintf('Latitud\n');
fprintf('Promedio: %.10f\n', lat_prom);
fprintf('Desv Std: %.10f\n', lat_std);
fprintf('RMSE: %.10f\n\n', lat_rmse);

fprintf('Longitud\n');
fprintf('Promedio: %.10f\n', lon_prom);
fprintf('Desv Std: %.10f\n', lon_std);
fprintf('RMSE: %.10f\n\n', lon_rmse);

fprintf('Altitud\n');
fprintf('Promedio: %.10f\n', alt_prom);
fprintf('Desv Std: %.10f\n', alt_std);
fprintf('RMSE: %.10f\n', alt_rmse);