clc; clear all; close all
load('dataLaser.mat')
load('dataTrayectoria.mat')
 
tam=size(dat1);
x = dat2(:, 1, 1);
y = dat2(:, 2);
th = dat2(:, 3);
mapa(50,50,1) = 0;
% Se anade los Landmarks 
landmarks = [3, 10; -2, -2; 6, -2];
% Datos del láser (720 filas modelo Hokuyo Utm-30lx-ew)
num_readings = 720;
angles = linspace(-135, 135, num_readings); % Ángulos del láser en grados
% Convertir ángulos a radianes
angles = deg2rad(angles);
% Configuración de la gráfica
figure(1)
xlabel('Coordenada X');
ylabel('Coordenada Y');
title('Pose del Robot y Datos del Láser');
 
acum_deter=0;
acum_minc=0;
% Preasignación de memoria para mejorar la eficiencia
pose = zeros(1, 3);
laser_x = zeros(1, num_readings);
laser_y = zeros(1, num_readings);
tam=size(dat1);

for i = 109:tam(1)
    % Datos de la pose del robot
    pose = [x(i), y(i), th(i)];
    % Generar rangos medidos desde la posición real del robot hacia los landmarks
    rangos_medidos = sqrt((landmarks(:, 1) - pose(1)).^2 + ...
                      (landmarks(:, 2) - pose(2)).^2);
    axis([-5 10 -5 15]);
    % Aplicar trilateración determinística
    ruido_aleatorio = 0.9; %+(rand(1, 3) - 0.5)*10;
    pose_estimada = trilateracion_deterministica(landmarks, rangos_medidos+ruido_aleatorio'   );              
    pose_estimada2 = trilateracion_minimos_cuadrados(landmarks, rangos_medidos+ruido_aleatorio'   );              
                   
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
    
    cad = strcat('Pose del Robot y Datos del Láser - Iter=', int2str(i));
    title(cad);
    figure(1)
    grid on;
    hold on;
    % Graficar la pose del robot
    plot(pose(1), pose(2), 'b*', 'MarkerSize', 1, 'LineWidth', 2);
    plot(pose_estimada(1), pose_estimada(2), 'mx', 'MarkerSize', 4, 'LineWidth', 2);
    plot(pose_estimada2(1), pose_estimada2(2), 'co', 'MarkerSize', 3, 'LineWidth', 2);
 
    % Graficar los landmarks  
    scatter(landmarks(:, 1), landmarks(:, 2), 100, 'k', 'filled', '^', 'DisplayName', 'Landmarks');
 
    Vec(i,:)=[pose(1) pose(2) pose_estimada(1) pose_estimada(2)...
               pose_estimada2(1) pose_estimada2(2)];
    Res_deter(i)= sqrt(([pose(1) pose(2)] - [pose_estimada(1) pose_estimada(2)])...
        *([pose(1) pose(2)] - [pose_estimada(1) pose_estimada(2)])');
    acum_deter=acum_deter+Res_deter(i);
    vec_acum_deter(i)=acum_deter;
    
    Res_minc(i)= sqrt(([pose(1) pose(2)] - [pose_estimada2(1) pose_estimada2(2)])...
        *([pose(1) pose(2)] - [pose_estimada2(1) pose_estimada2(2)])');
    acum_minc=acum_minc+Res_minc(i); 
    vec_acum_minc(i)=acum_minc;
    legend('Pose Real del Robot', 'Pose Deterministica','Pose Minimos Cuadrados');
    hold off;     
end
 
 
figure(3);
plot(vec_acum_deter, 'b', 'LineWidth', 2, 'DisplayName', 'Determinístico');
hold on;
plot(vec_acum_minc, 'r', 'LineWidth', 2, 'DisplayName', 'Mínimos Cuadrados');
xlabel('Iteración');
ylabel('Error Acumulado');
title('Comparacion: Deterministico vs. Minimos Cuadrados');
legend('Location', 'Best');
grid on;
hold off;
