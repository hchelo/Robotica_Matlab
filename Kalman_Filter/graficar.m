function graficar = graficar(fa,ruido,sinruido,estimado)
ruido = ruido(1,2:length(ruido)-1);    
sinruido = sinruido(1,2:length(sinruido)-1); % Primer valor no vale, inicial del arreglo
estimado = estimado(1,2:length(estimado)-1);

errEstim = ruido-estimado; % Error en la estimación
figure(fa)
subplot(311)
plot(sinruido,'b')
title(strcat('Estado x',num2str(fa),' Ideal - Sin Ruido'))
hold on
grid

subplot(312)
plot(ruido,'r')
title(strcat('Estado x',num2str(fa),' Ruidoso - Estado Estimado'))
hold on
plot(estimado,'k')
legend(strcat('x',num2str(fa),' Ruidoso'),strcat('x',num2str(fa),' Estimado'))
hold on
grid

subplot(313)
plot(errEstim)
title('Error Estimado (x Ruidoso- x Estimado)')
grid
hold on

