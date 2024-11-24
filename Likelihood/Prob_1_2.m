% Ejercicio 2 - Tarea 1 
% Robotica Movil UCN II/2023
% Marcelo Saavedra Alcoba
clc; clear all, close all
% Definir las ubicaciones
m = [10, 8];
t = [6, 3];
x0 = [12, 4];
x1 = [5, 7];

% Definir el rango de posiciones en la vecindad de las bases
x_range = 0:0.1:15;
y_range = 0:0.1:10;
[X, Y] = meshgrid(x_range, y_range);

% Calcular las distancias desde las bases
d0 = sqrt((X - x0(1)).^2 + (Y - x0(2)).^2);
d1 = sqrt((X - x1(1)).^2 + (Y - x1(2)).^2);

% Calcular la likelihood p(z | m) para todas las posiciones
likelihood = 1/(2*pi*sqrt(1*1.5))*exp(-0.5 * ((d0 - 3.9).^2 + (d1 - 4.5).^2));

% Graficar los puntos y la likelihood
figure;

% Puntos
scatter([m(1), t(1), x0(1), x1(1)], [m(2), t(2), x0(2), x1(2)], 100, 'filled');
hold on;

% Ubicaciones en la vecindad de las bases
scatter(X(:), Y(:), 5, 'y.');

% Superficie de likelihood
contour(X, Y, likelihood, 'LineColor', 'b');

% Etiquetas y leyenda
text(m(1), m(2), 'm', 'FontSize', 12, 'Color', 'k', 'HorizontalAlignment', 'right');
text(t(1), t(2), 't', 'FontSize', 12, 'Color', 'k', 'HorizontalAlignment', 'right');
text(x0(1), x0(2), 'x_0', 'FontSize', 12, 'Color', 'k', 'HorizontalAlignment', 'right');
text(x1(1), x1(2), 'x_1', 'FontSize', 12, 'Color', 'k', 'HorizontalAlignment', 'right');

xlabel('Posición en x');
ylabel('Posición en y');
title('Ubicaciones y Likelihood');
legend({'Puntos', 'Vecindad', 'Likelihood'}, 'Location', 'NorthEast');

hold off;

figure(2)
surf(likelihood);