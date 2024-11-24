function [x_hat, P, K] = update_step(x_hat_minus, P_minus, H, R)
    % Paso de actualización del filtro de Kalman

    % Residual de la medición
    z = measurement_step(x_hat_minus, H, R);
    y = z - H * x_hat_minus;

    % Ganancia de Kalman
    K = P_minus * H' / (H * P_minus * H' + R);

    % Actualización del estado estimado
    x_hat = x_hat_minus + K * y;

    % Actualización de la covarianza
    P = (eye(3) - K * H) * P_minus;
end