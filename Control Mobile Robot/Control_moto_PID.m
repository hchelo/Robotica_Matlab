clc;clear all; close all;

%% Parámetros del sistema
Ts = 0.1; % Tiempo de muestreo
t = 0:645; % Tiempo de simulación
d = 0.1; % Distancia entre el eje delantero y el trasero
pos = [1; 1; 0]; % Pose inicial del robot

%% Trayectoria
load('refi.mat')
x_ref=round(ref(:,1));
y_ref=round(ref(:,2));
tam=size(ref);
% Ganancias de control PID (ajusta estos valores según sea necesario)
kp_phi = 0.5;
ki_phi = 0.000001;
kd_phi = 0.0050;

kp_vr = 0.125;
ki_vr = 0.001;
kd_vr = 0.01;

% Estado inicial del PID
prev_error_phi = 0;
integral_phi = 0;
prev_error_vr = 0;
integral_vr = 0;


figure(1);
done = true;
for k=1:tam(1)
   % Referencia
    
    phi_ref = atan2(y_ref(k) - pos(2), x_ref(k) - pos(1));
    pos_ref = [x_ref(k); y_ref(k); phi_ref];
    
    % Llamada al controlador PID
    [alpha, vr, prev_error_phi, integral_phi, prev_error_vr, integral_vr] = controladorPID(pos, pos_ref, kp_phi, ki_phi, kd_phi, kp_vr, ki_vr, kd_vr, prev_error_phi, integral_phi, prev_error_vr, integral_vr);

    % Modelo robot móvil
    d_pos = [vr * cos(pos(3)); vr * sin(pos(3)); vr / d * tan(alpha)]; % [x';y';phi']
    ruido = 0; % ruido
    pos = pos + Ts * d_pos + randn(3, 1) * ruido; % Integración euler
    pos(3) = wrapToPi(pos(3)); % Mapeado al intervalo [-pi, pi].
    x(k) = pos(1,1);
    y(k) = pos(2,1);
    [k x(k) y(k) x_ref(k) y_ref(k)]
    
    % Cálculo del Mean Squared Error (MSE)
    error_pos = [x_ref(k) - x(k); y_ref(k) - y(k)];
    mse(k) = mean(error_pos(:).^2);  % MSE
    disp(['Mean Squared Error (MSE): ', num2str(mse(k))]);
    
    % Visualización del robot
    figure(1)
    plot(x, y, x_ref, y_ref, 'LineWidth', 2)
    hold on;
    plot(pos(1), pos(2), 'co', 'MarkerSize', 10, 'LineWidth', 1);
    quiver(pos(1), pos(2), 0.2 * cos(pos(3)), 0.2 * sin(pos(3)), 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'c');
    xlabel('x'), ylabel('y');
    axis([min(x_ref) - 0.5 max(x_ref) + 0.5 min(y_ref) - 0.5 max(y_ref) + 0.5])
    title('Seguimiento de trayectoria');
    legend('Salida', 'Referencia');
    grid on;
    hold off;
end
% Figura 2: Comportamiento del control
figure(2);
subplot(2,1,1);
plot(t(1,1:end-1), x_ref, 'LineWidth', 2, 'DisplayName', 'Referencia en x');
hold on;
plot(t(1,1:end-1), x, 'LineWidth', 2, 'DisplayName', 'Posición en x');
xlabel('Tiempo (s)'), ylabel('Posición en x');
title('Comportamiento del Control: Referencia vs Posición en x');
grid on
legend;

subplot(2,1,2);
plot(t(1,1:end-1), y_ref, 'LineWidth', 2, 'DisplayName', 'Referencia en y');
hold on;
plot(t(1,1:end-1), y, 'LineWidth', 2, 'DisplayName', 'Posición en y');
xlabel('Tiempo (s)'), ylabel('Posición en y');
title('Comportamiento del Control: Referencia vs Posición en y');
grid on
legend;

figure(3);plot(t(1,1:end-1), mse, 'LineWidth', 2);
xlabel('Tiempo (s)'), ylabel('MSE');
title('Comportamiento del MSE a lo largo del tiempo');
grid on;