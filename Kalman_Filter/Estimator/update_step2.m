function [x_hat, P, K] = update_step(x_hat_minus, P_minus, H, R)
    % Paso de actualizaci�n del filtro de Kalman

    % Residual de la medici�n
    z = measurement_step(x_hat_minus, H, R);
    y = z - H * x_hat_minus;

    % Ganancia de Kalman
    K = P_minus * H' / (H * P_minus * H' + R);

    % Actualizaci�n del estado estimado
    x_hat = x_hat_minus + K * y;

    % Actualizaci�n de la covarianza
    P = (eye(3) - K * H) * P_minus;
end