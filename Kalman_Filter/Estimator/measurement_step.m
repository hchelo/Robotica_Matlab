% Función de medición
function z = measurement_step(x,H,R)
    z = H * x + mvnrnd([0; 0;0], R)';
end