%% RRT* implementation
% MSaavedra
% reference: Hassan Umari / 
% Sampling-based Algorithms for Optimal Motion Planning,
% Emilio Frazzoli,Sertac Karaman
%Page 16
%%
close all
clear all
clc
xdim=100;
ydim=100;

fig=figure('name','RRT ','units','normalized','position',[0.15 0.06 0.7 .83]);
ax=gca; backColor=[222/255 222/255 239/255];
ax.Color = backColor;
axis([0 xdim 0 ydim])



%%
global eta d gama
eta=2; %growth distance (between nearset point towards the random point)
d=2; %2D space
gama=200; %check this????????????????????????
global obd
obd=0;
%% declaring obtacles (as points)
% x1 --> x2   y1-->y2    ob#=[x1 x2 y1 y2]
ob1=[0 20 60 80];plotObstacle(ob1);
ob2=[30 50 60 80];plotObstacle(ob2);
ob3=[60 80 60 80];plotObstacle(ob3);
ob4=[30 50  0  20];plotObstacle(ob4);
ob5=[60 80 30 50];plotObstacle(ob5);

ob13=[0 20  0  20];plotObstacle(ob13);
ob14=[60 80 0 20];plotObstacle(ob14);

ob6=[90 100 45 75];plotObstacle(ob6);

ob7=[0 20 30 50];plotObstacle(ob7);
ob8=[30 50 30 50];plotObstacle(ob8);

ob9=[80 88  18  20];plotObstacle(ob9);
ob10=[80 82  0  20];plotObstacle(ob10);
ob11=[98 100  0  20];plotObstacle(ob11);
ob12=[98 100  0  20];plotObstacle(ob12);




obstacles=[ob1;ob2;ob3;ob4;ob5;ob6;ob7;ob8;ob9;ob10;ob11;ob12;ob13;ob14];
obstacles2=obstacles;
%%
% Define el valor de dilatación
dilatacion = 3.5; % Por ejemplo, puedes ajustar este valor según tus necesidades

% Inicializar la matriz para almacenar los obstáculos dilatados
obstacles_dilatados = zeros(size(obstacles));

% Iterar sobre cada obstáculo
for i = 1:size(obstacles, 1)
    % Dilatar las coordenadas del obstáculo
    obstaculo_actual = obstacles(i, :);
    obstaculo_dilatado = obstaculo_actual + [-dilatacion, dilatacion, -dilatacion, dilatacion];
    
    % Asegurar que las coordenadas no sean negativas
    obstaculo_dilatado = max(obstaculo_dilatado, 0);
    
    % Almacenar el obstáculo dilatado en la matriz de obstáculos dilatados
    obstacles_dilatados(i, :) = obstaculo_dilatado;
end
obstacles=obstacles_dilatados;
% Plotear los obstáculos
% for i = 1:size(obstacles, 1)
%     x1 = obstacles(i, 1);
%     x2 = obstacles(i, 2);
%     y1 = obstacles(i, 3);
%     y2 = obstacles(i, 4);
%     plotObstacle([x1 x2 y1 y2]);
% end
% Visualizar los obstáculos dilatados
%disp("Obstáculos dilatados con valores no negativos:");
%disp(obstacles_dilatados);

%%
%% Target
Target=[90 10];radius=5; hold on; circle(Target(1),Target(2),radius);
x1 = 85;y1 = 10;str1 = 'Meta';text(x1,y1,str1,'Color','k','FontSize',15);
%% obtaining the random inital point
z=0;
while(z==0)
    
    V(1,:)=[xdim*rand ydim*rand];
    z=ObtacleFree(V(1,:),obstacles);
end
V(1,:)=[5 90];   %uncomment to force inital position

plot(V(1,1), V(1,2), '.k', 'MarkerSize',30)
%%


Cost(1,:)=[V 0];
vv=[V V];

iter=0; %loop counter,keeps track of the number of iterations
stp=0; %%stop condition initally zero, when it reaches target it is set to 1
s='Data';s1 = '1';
p=1;
while (1)
    
    iter=iter+1;
    if iter>=10
        iter=0;
        s = strcat(s,s1);
        %saveas(gcf,strcat(s,'.emf'))
        %saveas(gcf,strcat(s,'.fig'))
    end
    xrand=[xdim*rand ydim*rand];        %%pdf_Fazzoli_p16______________line3
    xnearest=Nearest(xrand,V);          %%pdf_Fazzoli_p16______________line4
    xnew=Steer(xnearest,xrand);         %%pdf_Fazzoli_p16______________line5
    
    if CollisionFree(xnearest,xnew,obstacles)%pdf_Fazzoli_p16__________line6
        V=[V;xnew];
        temp=(gama*log(length(V))/length(V))^(1/d);
        r=min(eta,temp);
        Xnear=Near(V,xnew,r);
        xmin=xnearest;
        
        
        costrow=find(any(Cost(:,1)==xnearest(1),2));
        cost=Cost(costrow,3)+norm(xnearest-xnew);
        cmin=cost;
        Cost=[Cost;[xnew cost]];
        oldc=cost;
        
        
        for indx=1:size(Xnear,1)
            xnear=Xnear(indx,:);
            costrow=find(any(Cost(:,1)==xnear(1),2));
            cxnear=Cost(costrow,3)+norm(xnear-xnew);
            
            if CollisionFree(xnear,xnew,obstacles) &&  (cxnear<cmin)
                xmin=xnear; cmin=cxnear;
                
            end
            
        end
        costrow=find(any(Cost(:,1)==xnew(1),2));
        Cost(costrow,:)=[];
        
        
        costrow=find(any(Cost(:,1)==xmin(1),2));
        cost=Cost(costrow,3)+norm(xmin-xnew);
        Cost=[Cost;[xnew cost]];
        vv=[vv;[xmin xnew]];
        plot([xmin(1) xnew(1)],[xmin(2) xnew(2)],'b') %% plot tree branch
        
        %% Rewire the tree
        for indx=1:size(Xnear,1)
            xnear=Xnear(indx,:);
            cost=cost+norm(xnew-xnear);
            costrow=find(any(Cost(:,1)==xnear(1),2));
            
            if CollisionFree(xnew,xnear,obstacles) &&  (cost<Cost(costrow,3))
                row=find(any(vv(:,3)==xnear(1),2));
                
                plot([vv(row,1) xnear(1)],[vv(row,2) xnear(2)],'Color',backColor)%delete old branch on the plot
                vv(row,:)=[];%delete old branch from the edges matrix
                vv=[vv;[xnew xnear]];
                
                %plot([xnew(1) xnear(1)],[xnew(2) xnear(2)],'r')%draw the new branch
                
            end
            
            
        end
        
        
        
        
        %%
        
        if norm(xnew-Target)<radius
            stp=1;
            row=find(any(vv(:,3)==xnew(1),2));
            
            while row>1
                
                parentx=vv(row,1);
                %plot([vv(row,1) vv(row,3)],[vv(row,2) vv(row,4)],'*r','LineWidth',3);
                row=find(any(vv(:,3)==parentx,2));
                ruta(p,:) = [vv(row,1) vv(row,2)]; p=p+1;
                ruta(p,:) = [vv(row,3) vv(row,4)]; p=p+1;
            end
            disp('encontrado!')
            
                       
            % Disctrtizando la ruta
            
            coordenadas_redondeadas = round(ruta);
            coordenadas_limpias = unique(coordenadas_redondeadas, 'rows', 'stable');
            puntos_discretizados = coordenadas_limpias(1:4:end, :);
            ruta2=puntos_discretizados;
            %figure(2); 
            plot(ruta2(:,1),ruta2(:,2),'*g','LineWidth',3)
            %plot([vv(row,1) vv(row,3)],[vv(row,2) vv(row,4)],':r','LineWidth',3);
            saveas(gcf,strcat('Resultado','.png'))
            saveas(gcf,strcat('Resultado','.fig'))
            
            return;
        end
        
        drawnow update
    end
end

