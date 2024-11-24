% Marcelo Saavedra
% Robotica II/2023
% Filtro extendido de Kalman
close all; clear all; clc,

global maxRange;
maxRange = 4; %Distancia máxima del laser %8

Tsim = 0.1; %Tiempo de Sampling de mi sistema
incr = 0;
%Inicializo Robots

robot.x = 0;
robot.y = 0; %es el robot segun odometria
robot.tita = 0;
%robot2 = robot; %es el robot real

Obs= [
    10.0 13.0;
    %11.0 9.0;
    12.0 11.0;
    %13.0 7.0;
    14.0 9.0;
    %15.0 5.0; 
    16.0 10.0;
    %17.0 6.0;
    18.0 13.0;
    %19.0 10.0
    ];
figure(1), hold on
axis([7 20 5 20]);
grid on
[Caras,Vertices] = conversionLandmarks(Obs);
plot(Obs(:,1),Obs(:,2),'ob');

%[X,Y] = ginput; %ingreso de los puntos de ruta
X= [8.0; 10.8; 20.0; 20.2];
Y= [9.0; 10.5; 10.0; 10.5];
     
 %no ingresar tantos puntos o la ruta se vuelve compleja para las pruebas
xmax = max(X); xmin = min(X);
xx = xmin:0.01:xmax;
yy = interp1(X,Y,xx,'spline');
CaminoReferencia = [xx' yy'];
plot(xx',yy','-r');

%Inicializo Robots en el punto de partida del camino generado
robot.x = X(1);
robot.y = Y(1);
robot.tita = atan2(Y(2) - Y(1), X(2) - X(1)); %es la pose del robot según la odometría. 

H1 = plotRobot(robot);
Umbraldist = 0.6;
%Genero un bucle para controlar el camino del robot
MAP_am = [];
Mean_Label = [];
vecPosReal = [];
vecPosicion = [];

%% Parametros para EKF
poseR = [robot.x;robot.y;robot.tita]; % Pose_0
Qv = [0.05 0 0; 0 0.05 0; 0 0 0.02]; %Covariance of the process noise
R = 1e4*eye(3); % Covariance of the measurement noise
%R = 10000* diag([1 1,pi]);
Sigma = 12*eye(3);

Ts = 0.1;
V = 0; W = 0; %ruidos 
F_jac = [1 0 -V*Ts*sin(poseR(3));0 1 V*Ts*cos(poseR(3));0 0 1]; %Jacobiano del proceso de medicion crta Robot. Localizacion
%%
robot2.x = poseR(1);
robot2.y = poseR(2); %es el robot segun odometria
robot2.tita = poseR(3);
title('Localizacion EKF','FontName','Calibri','FontSize', 14);
xlabel("X [m]",'FontName','Calibri','FontSize', 12);
ylabel("Y [m]",'FontName','Calibri','FontSize', 12);
set(gcf,'Color',[1 1 1])
H2 = plotRobot(robot2);
delete(H2);

%%
sigmVec12=0;
sigmVec21=0;
xESTIMADO = [];
kVector = [];
vecSigma=[];
for cont = 2:length(xx)-1
    delete(H1);
    %Sensing Characteristics
    Laser = MedicionConLaser(Vertices, Caras, robot);
    H3 = plotLaser(Laser,robot);
    
    Caracteristicas = Detector(Laser,robot);

    %% % ******************************* Starts Mapping *************************
    % Indentifying Characteristics by Euclidian Distance.
    if(~isempty(Caracteristicas))
        LabelMap = identificar(Caracteristicas,Umbraldist);
        NumPost = max(LabelMap(:,3));
    else
        LabelMap=0;
    end
    if LabelMap>0
        MeanPosition = media_inst(max(LabelMap(:,4)),LabelMap);
    else    
        MeanPosition =(20) * rand(1, 4) -10;
    end
    if(~isempty(MAP_am))
        MAP_am = dataAsociation(MeanPosition,MAP_am);
    else
        MAP_am = MeanPosition;
    end
    if(~isempty(MeanPosition) && ~isempty(Mean_Label))
        Mean_Label = etiquetar(Mean_Label,MeanPosition,MAP_am);
    else
        Mean_Label = identificar(MeanPosition(:,1:3),Umbraldist);
    end
    
    % Covariance compute for every iteration and group.
    NumPost = max(Mean_Label(:,3));
    
    %% ******************************* Starts Localization *************************
    % EKF Filter:
    % *************** TIME UPDATE: ****************
    % PREDICTION of the pose
    pose1 = poseR+Ts*[V*cos(poseR(3));V*sin(poseR(3));W]; % NON-LINEAR SYSTEM 
    F_jac = [1 0 -V*Ts*sin(poseR(3));0 1 V*Ts*cos(poseR(3));0 0 1]; %Jacobiano medicion  
    Sigma1 = F_jac*Sigma*F_jac'+Qv;
    
    % ************  MEASUREMENT UPDATE: ***********
    if (~isempty(MeanPosition))
        for k=1:size(MeanPosition,1)
            %DATA ASOCIATION:
            zt = lasercordlocal(MeanPosition(k,:),robot); % laser measure in local coords, robot coords. Real Data.
            if (LabelMap~=0)
                ztest = ztestimado(k,LabelMap,poseR); % laser measure based on estimated pose
            else 
                ztest=0;
            end 
                H = -1*eye(3);
            [Kt,poseR,Sigma] = measurement_update(H,Sigma,R,pose1,zt,ztest);
            xESTIMADO = [xESTIMADO Kt];
            sigmVec12 = [sigmVec12 Sigma(1,1)]; %cuidado con las poses
            sigmVec21 = [sigmVec21 Sigma(2,2)];
        end
        incr=incr+1;
    else
        poseR =  pose1;
    end
    
    [ poseR   Sigma]
    vecSigma = [vecSigma [Sigma(1,1);Sigma(2,2);Sigma(3,3)]];
    vecPosicion = [vecPosicion poseR(1:3,:)];
    plot(poseR(1),poseR(2),'.b')
    robot2.x = poseR(1);
    robot2.y = poseR(2);
    robot2.tita = poseR(3);
    H2 = plotRobot3(robot2);
    
    %% *********************************** Control ********************************
    
    % Controller reference
    Q = .01*eye(3,3);
    H = [cos(robot.tita) 0;sin(robot.tita) 0; 0 1];
    xref = xx(cont); yref = yy(cont);
    titaref = atan2(yy(cont+1) - robot.y,(xx(cont+1) - robot.x));
    deltax = (xref-robot.x)/Tsim + 0.1*(robot.x - xx(cont-1))/Tsim;
    deltay = (yref-robot.y)/Tsim + 0.3*(robot.y - yy(cont-1))/Tsim;
    sigma1 = (Q(1,1))^2;
    sigma2 = (Q(2,2))^2;
    sigma3 = (Q(3,3))^2;
    Control(2) = (titaref-robot.tita)/Tsim;
    Control(1) = (2*deltax*sigma2^2*cos(robot.tita))/(sigma2^2*cos(2*robot.tita) - sigma1^2*cos(2*robot.tita) + sigma1^2 + sigma2^2) + (2*deltay*sigma1^2*sin(robot.tita))/(sigma2^2*cos(2*robot.tita) - sigma1^2*cos(2*robot.tita) + sigma1^2 + sigma2^2);
    if Control(1) > 0.3
        Control(1) = 0.3;
    end
    if abs(Control(2)) > 1
        if (Control(2) <= 0)
            Control(2) = -1;
        else
            Control(2) = 1;
        end
    end
    V = Control(1);
    W = Control(2);
    robot.x = robot.x + Tsim*V*cos(robot.tita)+ 0.003*abs(rand(1,1));
    robot.y = robot.y + Tsim*V*sin(robot.tita)+ 0.003*abs(rand(1,1));
    robot.tita = robot.tita + Tsim*W;
    
    vecPosReal = [vecPosReal [robot.x;robot.y;robot.tita]];
    H1 = plotRobot(robot);
    
    pose_ref_x(cont-1) = robot.x; pose_ref_y(cont-1) = robot.y; pose_ref_t(cont-1) = robot.tita;
    pose_est_x(cont-1) = poseR(1,1); pose_est_y(cont-1) = poseR(2,1); pose_est_t(cont-1) = poseR(3,1);

    grid on
    pause(Tsim);
    delete(H3);
    delete(H2);

end


%% **********************  RESULTADOS ******************************
% Vector final de estados y su Covarianza
disp('x^+  --- P+     ')
[ poseR   Sigma]

error_x=vecPosicion(1,:)-vecPosReal(1,:);
error_y=vecPosicion(2,:)-vecPosReal(2,:);
error_tita=vecPosicion(3,:)-vecPosReal(3,:);
figure(3);
% Subtrama para el error en x
subplot(3, 1, 1); % 2 filas, 1 columna, primera subtrama
plot(error_x,'.b');
axis([0 1200 -1 5]);
title('Error en la Coordenada X');
xlabel('Tiempo');
ylabel('Error');
grid on
% Subtrama para el error en y
subplot(3, 1, 2); % 2 filas, 1 columna, segunda subtrama
plot(error_y,'.b');
axis([0 1200 -1 3]);
title('Error en la Coordenada Y');
xlabel('Tiempo');
ylabel('Error');
grid on
% Subtrama Orientacion
subplot(3, 1, 3); % 2 filas, 1 columna, segunda subtrama
plot(error_tita,'.b');
axis([0 1200 -1 3]);
title('Error en la Orientacion Theta');
xlabel('Tiempo');
ylabel('Error');
grid on

figure(4);
% Subtrama para el error en x
subplot(3, 1, 1); % 2 filas, 1 columna, primera subtrama
plot(error_x,'.b');
axis([0 1200 -1 5]);
title('Error en la Coordenada X');
xlabel('Tiempo');
ylabel('Error');
grid on
% Subtrama para el error en y
subplot(3, 1, 2); % 2 filas, 1 columna, segunda subtrama
plot(error_y,'.b');
axis([0 1200 -1 3]);
title('Error en la Coordenada Y');
xlabel('Tiempo');
ylabel('Error');
grid on
% Subtrama Orientacion
subplot(3, 1, 3); % 2 filas, 1 columna, segunda subtrama
plot(error_tita,'.b');
axis([0 1200 -1 3]);
title('Error en la Orientacion Theta');
xlabel('Tiempo');
ylabel('Error');
grid on

%% Grafico Covarianza

cov_x=vecSigma(1,:);
error_y=vecPosicion(2,:)-vecPosReal(2,:);
error_tita=vecPosicion(3,:)-vecPosReal(3,:);

figure(4);
% Subtrama para el error en x
subplot(3, 1, 1); % 2 filas, 1 columna, primera subtrama
plot(vecSigma(1,:),'.b');
%axis([0 1200 -1 5]);
title('Varianza en X');
xlabel('Tiempo');
ylabel('Error');
grid on
% Subtrama para el error en y
subplot(3, 1, 2); % 2 filas, 1 columna, segunda subtrama
plot(vecSigma(2,:),'.b');
%axis([0 1200 -1 3]);
title('Varianza en Y');
xlabel('Tiempo');
ylabel('Error');
grid on
% Subtrama Orientacion
subplot(3, 1, 3); % 2 filas, 1 columna, segunda subtrama
plot(vecSigma(3,:),'.b');
%axis([0 1200 -1 3]);
title('Varianza en Theta');
xlabel('Tiempo');
ylabel('Error');
grid on