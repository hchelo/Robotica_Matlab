function pose_estimada = trilateracion_minimos_cuadrados(landmarks, rangos_medidos)
    num_landmarks = size(landmarks, 1);
    
    % Verificar si hay al menos tres landmarks
    if num_landmarks < 3
        error('Se requieren al menos tres landmarks para la trilateración.');
    end
    
    % Función objetivo
    fun = @(pose) sum((sqrt((landmarks(:, 1) - pose(1)).^2 + (landmarks(:, 2) - pose(2)).^2) - rangos_medidos).^2);
    
    % Estimación inicial de la pose (usando el primer landmark)
    pose_inicial = landmarks(1, :);
    
    % Aplicar minimización de la función objetivo
    pose_estimada = fminsearch(fun, pose_inicial);
end