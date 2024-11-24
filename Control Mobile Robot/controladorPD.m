function [alpha, vr, prev_error_phi, prev_error_vr] = controladorPD(pos, pos_ref, kp_phi, kd_phi, kp_vr, kd_vr, prev_error_phi, prev_error_vr)
    % Errores con respecto a la referencia
    e = pos_ref - pos; % errores pose
    e(3) = wrapToPi(e(3)); % Mapeado al intervalo [-pi, pi].

    % Control PD para la orientación
    error_phi = e(3);
    derivative_phi = error_phi - prev_error_phi;
    alpha = kp_phi * error_phi + kd_phi * derivative_phi;

    % Control PD para la velocidad lineal
    error_vr = sqrt(e(1)^2 + e(2)^2);
    derivative_vr = error_vr - prev_error_vr;
    vr = kp_vr * error_vr + kd_vr * derivative_vr;

    % Actualizar errores para la próxima iteración
    prev_error_phi = error_phi;
    prev_error_vr = error_vr;
end