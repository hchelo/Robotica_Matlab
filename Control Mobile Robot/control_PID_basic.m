clear all; close all; clc
%Ejercicio 4
Ts = 0.1; % Tiempo de muestreo
t = 0:Ts:50; % Tiempo de simulación
d = 0.1; % Distancia entre el eje delantero y el trasero
pos = [1; 1; 0]; % Pose inicial del robot

% Trayectorias
f = 2*pi/30;
% Forma circular
x_ref = 1 + 0.7*sin(f*t);
y_ref = 1 + 0.7*cos(f*t);
% ------------------------------------------------------------------------
% Ganancias de control (Controlador proporcional)
kp_phi = 12;
kp_vr = 3;
% ------------------------------------------------------------------------

done = true;
figure(1);
hold on
grid on
xlabel("x"), ylabel("y")
title("Seguimiento de trayectoria")

% Solo dibujar la trayectoria de referencia una vez
plot(x_ref, y_ref, 'LineWidth', 1, 'color', 'r'); % Trayectoria de referencia

% Inicializar las variables de la leyenda con valores NaN
robot_plot = plot(NaN, NaN, 'LineWidth', 2, 'color', 'b'); % Solo para la leyenda
arrow_plot = plot(NaN, NaN, 'LineWidth', 1.5, 'color', 'k'); % Solo para la leyenda

% Crear la leyenda fuera del bucle

for k = 1:length(t)
    % Referencia
    phi_ref = atan2(y_ref(k)-pos(2), x_ref(k)-pos(1));
    pos_ref = [x_ref(k); y_ref(k); phi_ref];
    % ------------------------------------------------------------------------

    % Errores con respecto a la referencia
    e = pos_ref - pos; % errores pose
    mse = mean(e.^2); % MSE
    e(3) = wrapToPi(e(3)); % Mapeado al intervalo [-pi, pi].
    % ------------------------------------------------------------------------

    % Señales de control
    alpha = e(3)*kp_phi; % Control de orientación
    vr = sqrt(e(1)^2 + e(2)^2)*kp_vr; % Control en x,y
    % ------------------------------------------------------------------------

    if done
        vr = vr*sign(cos(e(3))); %
        e(3) = atan(tan(e(3))); % Mapeo al intervalo [-pi/2, pi/2]
        alpha = e(3)*kp_phi; % actualizar control de orientación
    end

    % Limitaciones físicas de velocidad y ángulo de giro
    if abs(alpha) > pi/6
        alpha = pi/6 * sign(alpha);
    end
    if abs(vr) > 0.7
        vr = 0.7 * sign(vr);
    end
    % ------------------------------------------------------------------------

    % Modelo robot móvil
    d_pos = [vr*cos(pos(3)); vr*sin(pos(3)); vr/d*tan(alpha)]; % [x';y';phi']
    ruido = 0.001; % ruido
    pos = pos + Ts*d_pos + randn(3,1)*ruido; % Integración Euler
    pos(3) = wrapToPi(pos(3)); % Mapeado al intervalo [-pi, pi].
    x(k) = pos(1);
    y(k) = pos(2);
    % ------------------------------------------------------------------------

    % Dibujar círculo y flecha
    robot_radius = 0.05; % Radio del círculo que representa al robot
    theta = linspace(0, 2*pi, 50); % Coordenadas del círculo
    circle_x = robot_radius * cos(theta) + pos(1);
    circle_y = robot_radius * sin(theta) + pos(2);

    % Borra el robot anterior
    if k > 1
        delete(robot_plot);
        delete(arrow_plot);
    end

    % Dibuja el robot
    robot_plot = fill(circle_x, circle_y, 'b'); % Círculo del robot
    arrow_plot = quiver(pos(1), pos(2), 0.1*cos(pos(3)), 0.1*sin(pos(3)), ...
                        'MaxHeadSize', 2, 'color', 'k', 'LineWidth', 1.5); % Flecha

    % Trazo de seguimiento
    plot(x(1:k), y(1:k), 'LineWidth', 2, 'color', 'b')

    pause(0.05);
end
legend("Trayectoria de referencia", "Posición del robot");
