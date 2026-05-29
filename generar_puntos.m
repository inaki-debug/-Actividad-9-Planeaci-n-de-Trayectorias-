clc; clear; close all;

% Configuración del mapa en metros
max_x_metros = 12; 
max_y_metros = 12;

% Cargar imagen
[file, path] = uigetfile({'*.jpg;*.png;*.jpeg;*.bmp'}, 'Selecciona imagen');
if isequal(file,0)
    return;
end

% Voltear imagen para alinearla con el origen del robot
img = flipud(imread(fullfile(path, file))); 

% Configurar figura y plano cartesiano
figure('Name', 'Generador de Waypoints', 'NumberTitle', 'off');
imshow(img, 'XData', [0 max_x_metros], 'YData', [0 max_y_metros]);
axis on;
axis([0 max_x_metros 0 max_y_metros]);
set(gca, 'YDir', 'normal'); 
hold on;
grid on; 

% Inicializar variables
x_list = [];
y_list = [];

% Instrucciones en consola
disp('Haz clic para agregar puntos. Presiona ENTER para finalizar.');

% Capturar clics
while true
    [x, y, button] = ginput(1);
    if isempty(button) % Termina al presionar ENTER
        break;
    end
    
    % Redondear coordenadas a 1 decimal
    x_val = round(x, 1);
    y_val = round(y, 1);
    
    x_list = [x_list; x_val];
    y_list = [y_list; y_val];
    
    % Dibujar punto y línea de trayectoria
    plot(x_val, y_val, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    if length(x_list) > 1
        plot(x_list(end-1:end), y_list(end-1:end), 'b-', 'LineWidth', 1);
    end
end

% Imprimir resultado final
fprintf('\nwaypoints = [');
for i = 1:length(x_list)
    if i == length(x_list)
        fprintf('%g,%g', x_list(i), y_list(i));
    else
        fprintf('%g,%g; ', x_list(i), y_list(i));
    end
end
fprintf('];\n\n');

hold off;