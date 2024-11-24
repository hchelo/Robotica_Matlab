function pose_estimada = trilateracion_deterministica(landmarks, rangos_medidos)
    num_landmarks = size(landmarks, 1);
    
    % Verificar si hay al menos tres landmarks
    if num_landmarks < 3
        error('Se requieren al menos tres landmarks para la trilateración.');
    end
    
    % Crear la matriz A y el vector b
    A = 2 * (landmarks(2:end, :) - landmarks(1, :));
    b = (rangos_medidos(2:end).^2 - rangos_medidos(1)^2 + ...
         landmarks(1, 1)^2 + landmarks(1, 2)^2 - landmarks(2:end, 1).^2 - landmarks(2:end, 2).^2)';
    
    % Resolver el sistema de ecuaciones lineales
    pose_estimada = -pinv(A) * b';
end