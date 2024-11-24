clc; clear all; close all

load('dataLaser.mat')
load('dataTrayectoria.mat')

tam=size(dat1);
x = dat2(:, 1, 1);
y = dat2(:, 2);
th = dat2(:, 3);
mapa(50,50,1) = 0;

% Datos del láser (720 filas modelo Hokuyo Utm-30lx-ew)
num_readings = 720;
angles = linspace(-135, 135, num_readings); % Ángulos del láser en grados

% Convertir ángulos a radianes
angles = deg2rad(angles);

% Configuración de la gráfica
figure(1)
axis([-20 20 -20 20]);
xlabel('Coordenada X');
ylabel('Coordenada Y');
title('Pose del Robot y Datos del Láser');
grid on;

hold on;

% Preasignación de memoria para mejorar la eficiencia
pose = zeros(1, 3);
laser_x = zeros(1, num_readings);
laser_y = zeros(1, num_readings);

for i = 1:tam(1)
    % Datos de la pose del robot
    pose = [x(i), y(i), th(i)];
    
    for j = 1:720
        if (dat1(i, j) < 8) 
            dat11(i, j) = 0;
        elseif (dat1(i, j) > 14)
            dat11(i, j) = 0;
        else    
            dat11(i, j) = dat1(i, j);
        end 
    end
    
    % Datos del láser
    laser_data = dat11(i, :); 
    
    % Transformando datos láser a coordenadas (x, y) desde el robot
    laser_x = laser_data .* cos(angles + pose(3)) + pose(1);
    laser_y = laser_data .* sin(angles + pose(3)) + pose(2);
    figure(1)
    cad = strcat('Pose del Robot y Datos del Láser - Iter=', int2str(i));
    title(cad);
    % Graficar la pose del robot
    plot(pose(1), pose(2), 'bo', 'MarkerSize', 10, 'LineWidth', 1);
    
    % Graficar los datos del láser 
    plot(laser_x, laser_y, 'r.', 'MarkerSize', 5);
    
    % Graficar la orientación del robot
    quiver(pose(1), pose(2), 1 * cos(pose(3)), 1 * sin(pose(3)), 'b', 'LineWidth', 1);
   
    for mm=1:720
        if i>2
            laser_rx(mm)=round(laser_x(mm));
            laser_ry(mm)=round(laser_y(mm));
            if laser_rx(mm)==0 &&  laser_y(mm)==0
                mapa(30-laser_ry(mm),30+laser_rx(mm),i)=0;
            else
                mapa(30-laser_ry(mm),30+laser_rx(mm),i)=1;
            end    
        end
    end
    if i>2
        mapa(:,:,i)=(mapa(:,:,i)+mapa(:,:,i-1));
        %figure(2)
        %imshow(1-mapa(:,:,i),[])
    end
    
end

hold off;
% Ejercicio 2 Sensores - Tarea 1 
% Robotica Movil UCN II/2023
% Marcelo Saavedra Alcoba
% mapa estadistico 
for i=1:50
    for j=1:50
        if mapa(i,j,end)>15
            mapi(i,j) = mapa(i,j,end);
        else
            mapi(i,j) =0;
        end
    end
end
figure(2)
imshow(1-mapi,[])
title('Mapa Estadistico desde Lidar');