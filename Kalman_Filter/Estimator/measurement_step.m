% Funci�n de medici�n
function z = measurement_step(x,H,R)
    z = H * x + mvnrnd([0; 0;0], R)';
end