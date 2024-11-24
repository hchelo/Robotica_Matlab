% Parámetros iniciales
clear all; close all; clc
T = 1;  % Paso de tiempo
Q = [2 0 0; 0 2 0; 0 0 2];  % Covarianza del ruido de proceso
R = [0.4 0 0; 0 0.2 0; 0 0 0.1];  % Covarianza del ruido de medición

% Condiciones iniciales
x_hat = [2; 2; 0];  % Estado inicial
P = 20 * eye(3);  % Covarianza inicial
num_iteraciones=50;
% Matrices de modelado
H = eye(3);  % Matriz de medición (en este caso, la medición es directa)
u = zeros(1, num_iteraciones);  % Vector de entrada prealocado

% Inicializar matrices para almacenar resultados
x_hat_history = zeros(3, num_iteraciones);
P_history = zeros(3, 3, num_iteraciones);
w=[0.2 -0.4 0.1]';
% Iteraciones del filtro de Kalman Extendido
for k = 1:num_iteraciones
    u = [1 pi/6 * sin(0.1 * k) 0]';  % Vector de entrada
    
    % Paso de predicción
    %[x_hat_minus, P_minus, F] = prediction_step2(x_hat, P, u(k), T, Q, k);
    %===
    x_hat_minus = x_hat + T *u + w ;

    % Actualización de la matriz jacobiana F
    F = eye(3) + T * [0, 0, -u(1) * sin(x_hat(3));
                     0, 0, u(1) * cos(x_hat(3));
                     0, 0, 0];

    % Predicción de la covarianza
    P_minus = F * P * F' + Q;
    %===
    % Paso de actualización
    [x_hat, P, K] = update_step2(x_hat_minus, P_minus, H, R);
    
    % Almacenar resultados
    x_hat_history(:, k) = x_hat;
    x_hat_minusH(:, k) = x_hat_minus;
    P_history(:, :, k) = P;
    
    % Mostrar resultados
    disp(['Iteración ' num2str(k)]);
    disp('Estado estimado:');
    disp(x_hat);
    disp('Covarianza estimada:');
    disp(P);
    disp('---');
    
    % Graficar la trayectoria real y estimada
    figure(1)
    plot(x_hat_history(1, 1:k), x_hat_history(2, 1:k), 'og', 'LineWidth', 2);
    hold on
    plot(x_hat_minusH(1, 1:k), x_hat_minusH(2, 1:k), '*r', 'LineWidth', 2);

    xlabel('Posición en X');
    ylabel('Posición en Y');
    legend('Estimada','real');
    title(['Iteración ' num2str(k)]);
    grid on;
end