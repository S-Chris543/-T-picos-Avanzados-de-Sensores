%{
Práctica 01. Sistema de Posicionamiento Global GPS:
Adquisición de datos GPS a 1 Hz durante 15 minutos

- Christian Emmanuel Castruita Alaniz: Desarrollo principal del código GPS y geocerca.
- Del Hoyo Gómez Karla Stephanie: Pruebas de funcionamiento y documentación.
- Pablo David Sánchez García: Apoyo en cálculos y optimización de la geocerca.

El código de MATLAB analiza datos de sensores GPS mediante la carga de conjuntos de 
datos de los siguientes entornos; exterior, salón y entre calles para extraer 
variables de latitud longitud y altitud para realizar el cálculo preciso de la 
distancia total que detectó el GPS y así, genera visualizaciones como mapas 
geográficos de trayectoria satélite y gráficas de la variación de altitud por 
cada muestra recolectada.
%}

% ------------------------------------------------------------------------------------------------

% Archivos
archivos = {'CampoAbierto.m..mat', 'EntreCalles.m..mat', 'Salon.mat'};
nombres = {'Campo Abierto', 'Entre Calles', 'Salón'};

for i = 1:length(archivos)
    if exist(archivos{i}, 'file')
        fprintf('Procesando: %s...\n', archivos{i});
        load(archivos{i}); % cargar posiciones
        N = height(Position);
        
        % Primer gráfica - trayectoria
        figure('Name', [nombres{i} ' - Mapa'], 'Color', 'w');
        
        %formato
        geoplot(Position.latitude, Position.longitude, '-b', 'LineWidth', 2.5); 
        geobasemap('satellite'); 
      
        title(['Trayectoria GPS - ' nombres{i}], ...
              'Color', 'k', 'FontSize', 14, 'FontWeight', 'bold');

        % segunda grafica - altitud
        figure('Name', [nombres{i} ' - Altitud'], 'Color', 'w');
        plot(1:N, Position.altitude, '-b', 'LineWidth', 1.5);
        
        %formato
        grid on;
        ax = gca;
        ax.Color = 'w';          
        ax.XColor = 'k';          
        ax.YColor = 'k';          
        ax.FontSize = 12;         
        ax.FontWeight = 'bold';
        ax.GridColor = 'k';   
        ax.GridAlpha = 0.2;       
        
        title(['Variación de Altitud - ' nombres{i}], 'Color', 'k', 'FontSize', 14);
        xlabel('Muestra', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('Altitud (m)', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');

        drawnow;
    else
        warning('No encontré el archivo: %s', archivos{i});
    end
end
end
