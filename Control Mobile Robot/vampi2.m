clear all
close all
clc
% Parámetros del sistema
Ts = 0.1;                % Tiempo de muestreo
t = 0:Ts:50;             % Tiempo de simulación
d = 0.1;                 % Distancia entre el eje delantero y el trasero
pos = [0; 0; 0];         % Pose inicial del robot

% Trayectorias Senosoidal
f = 2*pi/30;
x_ref = 1 + 0.7 * sin(f * t);
y_ref = 1 + 0.7 * cos(f * t);

% Ganancias de control PD
kp_phi = 1;
kd_phi = 0.1;

kp_vr = 1;
kd_vr = 0.1;

% Estado inicial del PD
prev_error_phi = 0;
prev_error_vr = 0;

% Vectores para almacenar las posiciones y errores
x = zeros(size(t));
y = zeros(size(t));
error_phi_vec = zeros(size(t));
error_vr_vec = zeros(size(t));
error_x_vec = zeros(size(t));  % Error en el eje X
error_y_vec = zeros(size(t));  % Error en el eje Y

% Bucle principal
figure(1);

for k = 1:length(t)
    % Referencia
    phi_ref = atan2(y_ref(k) - pos(2), x_ref(k) - pos(1));
    pos_ref = [x_ref(k); y_ref(k); phi_ref];

    % Llamada al controlador PD
    [alpha, vr, prev_error_phi, prev_error_vr] = controladorPD(pos, pos_ref, kp_phi, kd_phi, kp_vr, kd_vr, prev_error_phi, prev_error_vr);
   
    % Modelo robot móvil
    d_pos = [vr*cos(pos(3)); vr*sin(pos(3)); vr/d*tan(alpha)]; % [x';y';phi']
    ruido = 0.001; % ruido
    pos = pos + Ts*d_pos + randn(3,1)*ruido; % Integración euler
    
    % Almacenar posiciones y errores
    x(k) = pos(1);
    y(k) = pos(2);
    error_phi_vec(k) = phi_ref - pos(3);
    error_vr_vec(k) = norm(pos_ref(1:2) - pos(1:2));

    % Calcular errores en los ejes X e Y
    error_x_vec(k) = x_ref(k) - pos(1);  % Error en el eje X
    error_y_vec(k) = y_ref(k) - pos(2);  % Error en el eje Y

    % Visualización del robot
    subplot(1, 2, 1);
    plot(x_ref, y_ref, 'LineWidth', 2);
    hold on;
    plot(pos(1), pos(2), 'co', 'MarkerSize', 10, 'LineWidth', 1);
    quiver(pos(1), pos(2), 0.2*cos(pos(3)), 0.2*sin(pos(3)), 'LineWidth', 2, 'MaxHeadSize', 0.5,'Color', 'c');
    xlabel('x'), ylabel('y');
    axis([min(x_ref)-0.5 max(x_ref)+0.5 min(y_ref)-0.5 max(y_ref)+0.5])
    title('Seguimiento de trayectoria');
    legend('Referencia', 'Posición Actual');
    grid on;
    hold off;

    % Visualización de errores en el eje X y Y
    subplot(2, 2, 2);
    plot(t(1:k), error_x_vec(1:k), 'LineWidth', 2);
    xlabel('Tiempo (s)'), ylabel('Error en el Eje X');
    title('Error en el Eje X');
    grid on;

    subplot(2, 2, 4);
    plot(t(1:k), error_y_vec(1:k), 'LineWidth', 2);
    xlabel('Tiempo (s)'), ylabel('Error en el Eje Y');
    title('Error en el Eje Y');
    grid on;

    drawnow;
    disp('P_phi D_phi P_v D_v')
    [alpha vr] 
end
