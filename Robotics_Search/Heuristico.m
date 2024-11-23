clear all;
close all;
radius=4;
fileID = fopen('heuristico.txt', 'w');
fclose(fileID);
for j=1:10
    % Parámetros del sistema
    
    fileID = fopen('heuristico.txt', 'a');
    encontrado=0;
    close all;
    Ts = 0.1;                % Tiempo de muestreo
    t = 0:Ts:60;             % Tiempo de simulación
    pos = [-5; -5];            % Pose inicial del robot
    ruta = pos';           % Almacena las posiciones visitadas
    Mat = zeros(11,11);
    %Mat(10,8) = -2;
    
    % Cargar imagen
    img = imread('contam_arriba.png');
    %figure(3);
    %imshow(img);
    
    % Parámetros de movimiento
    vel_lin = 0.5;                % Velocidad lineal constante
    dist = 0;
    theta=0;
    % Preconfigurar las figuras para evitar configurarlas dentro del bucle
    
    %figure('Position', [100, 100, 1250, 520]);
    figure('Name', 'Heuristico','Position', [100, 100, 1250, 520]);
    subplot(1, 2, 1);  % Primera subfigura
    hold on;
    xlabel('x'), ylabel('y');
    title('Movimiento Heuristico del Robot');
    axis([-6 6 -6 6]);
    grid on;
    rojo =0;
    %figure(2);
    %imshow(flipud(Mat'), []);
    

    tic
    % Bucle principal
    vec_encon(j)=0;
    ban=0;
    for k = 1:length(t)
        
        % Generar movimiento aleatorio

       if rojo<350000 && ban==0     
            direccion = rand * 2 * pi;    % Dirección aleatoria entre 0 y 2*pi
            posa = pos;
            new_pos = pos + vel_lin * [cos(direccion); sin(direccion)];
            disp('ban0')
            ban
            % Actualizar posición del robot dentro de los límites
            if (new_pos(1) >= -5 && new_pos(1) <= 5) && (new_pos(2) >= -5 && new_pos(2) <= 5)
                pos = new_pos;
            else
                pos = pos - round(pos - new_pos);
            end
            
          
            %k
        else
            ban=1;
            
            % Actualizar posición del robot dentro de un radio especificado

                theta = theta + vel_lin * Ts / radius;  % Incrementar ángulo
                new_pos = posa + radius * [cos(theta); sin(theta)];  % Calcular nueva posición

                % Actualizar posición del robot
                pos = new_pos;
                disp('ban1')
                ban
           
            %direccion = rand * 2 * pi;    % Dirección aleatoria entre 0 y 2*pi
            %new_pos = pos + vel_lin * [cos(direccion); sin(direccion)];
            %posa = new_pos;
        end
          % Verificar y actualizar la posición en la matriz
            xi = round(pos(1)) + 6;
            yi = round(pos(2)) + 6;
            if xi>11
               xi=11; 
            end    
            if xi<1
               xi=1; 
            end   
            if yi>11
               yi=11; 
            end    
            if yi<1
               yi=1; 
            end 
            if Mat(xi, yi) == 0 && dist<10
                ruta = [ruta; pos'];
                dist = sqrt(sum((pos - new_pos).^2));
                
                % Visualización del robot y la ruta
                %figure(1);
                subplot(1, 2, 1);  % Primera subfigura
                plot(pos(1), pos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1);
                %pause(dist)
                quiver(pos(1), pos(2), 0.2 * cos(direccion), 0.2 * sin(direccion), 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'c');
                plot(ruta(:, 1), ruta(:, 2), 'b', 'MarkerSize', 10);
                plot(pos(1), pos(2), 'co', 'MarkerSize', 10, 'LineWidth', 1);
                drawnow;
                
                Mat(xi, yi) = Mat(xi, yi) + 1;
                
                % Actualizar imagen
                rojo = sum(sum(img((12 - yi) * 100 - 99 : (12 - yi) * 100, xi * 100 - 99 : xi * 100, 1)));
                img((12 - yi) * 100 - 99 : (12 - yi) * 100, xi * 100 - 99 : xi * 100, 1:3) = 0;
                
                %figure(2);
                %imshow(flipud(Mat'), []);
                
                %figure(3);
                subplot(1, 2, 2);  % Segunda subfigura
                imshow(img);
                [xi yi rojo]
            elseif rojo >1800000
                disp('Encontrado...');
                encontrado=1;
                rojo = sum(sum(img((12 - yi) * 100 - 99 : (12 - yi) * 100, xi * 100 - 99 : xi * 100, 1)));
                img((12 - yi) * 100 - 99 : (12 - yi) * 100, xi * 100 - 99 : xi * 100, 1:3) = 0;
                [xi yi rojo]
                %figure(1);
                subplot(1, 2, 1);  % Primera subfigura
                plot(pos(1), pos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1);
                quiver(pos(1), pos(2), 0.2 * cos(direccion), 0.2 * sin(direccion), 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'c');
                vec_encon(j)=1;
                subplot(1, 2, 2);  % Segunda subfigura
                imshow(img);
                break;
            end
    end
    vec(j)=toc;
    % Escribir las variables en la misma línea en el archivo
    fprintf(fileID, ' %d %.2f %d\n', encontrado, vec(j),j);
    fclose(fileID);
end
[mean(vec) sum(vec_encon)]