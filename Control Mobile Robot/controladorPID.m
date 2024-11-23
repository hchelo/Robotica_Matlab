function [alpha, vr, prev_error_phi, integral_phi, prev_error_vr, integral_vr] = controladorPID(pos, pos_ref, kp_phi, ki_phi, kd_phi, kp_vr, ki_vr, kd_vr, prev_error_phi, integral_phi, prev_error_vr, integral_vr)
    % Errores con respecto a la referencia
    e = pos_ref - pos; % errores pose
    e(3) = wrapToPi(e(3)); % Mapeado al intervalo [-pi, pi].

    % Control PID para la orientación
    error_phi = e(3);
    integral_phi = integral_phi + error_phi;
    derivative_phi = error_phi - prev_error_phi;

    alpha = kp_phi * error_phi + ki_phi * integral_phi + kd_phi * derivative_phi;

    % Control PID para la velocidad lineal
    error_vr = sqrt(e(1)^2 + e(2)^2);
    integral_vr = integral_vr + error_vr;
    derivative_vr = error_vr - prev_error_vr;

    vr = kp_vr * error_vr + ki_vr * integral_vr + kd_vr * derivative_vr;

    % Actualizar errores y términos integrales para la próxima iteración
    prev_error_phi = error_phi;
    prev_error_vr = error_vr;
end
