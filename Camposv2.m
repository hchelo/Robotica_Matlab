clear all; close all; clc
load M5.mat
 M=M4;
 %figure(1); imshow(M,[]);
 writerObj = VideoWriter('path.mp4', 'MPEG-4');
 writerObj.FrameRate = 30;
 open(writerObj);
 
 figure(1);surf(M'), view(90,90)
 
    for i=1:50
      for j=1:50 
          if M(i,j)==2
             x=i;    
             y=j; 
             M(i,j)=0;
          end
           if M(i,j)==-1
             x1=i;    
             y1=j;   
             M(i,j)=0;
          end
      end
    end  

% Elemento Estructurante
se = strel('square',5);
%M3=IMDILATE(M,se);
M3=imdilate(M,se);
%figure(2); imshow(M3,[]);
figure(2);surf(M3'), view(90,90)
pause(1)
M3(x,y)=2;
M3(x1,y1)=-1;
M=M3;
     UA=0;
     UR(50,50)=0;
     U=0;
     
    
     K=3;
    for i=1:50
      for j=1:50 
         if M(i,j)==1
             UR(i,j)=1/2*K * (sqrt(((i-x)^2 + (j-y)^2)))+K;
             %UR(i,j)=K;
         end  
             UA(i,j)= 1/2*K * (sqrt(((i-x)^2 + (j-y)^2)));
             
      end
    end
  
   

    figure(3); surf(UA')
    title('Potencial de Atraccion')
    pause(1)
    
    figure(4); surf(UR')
    title('Potencial de Repulsion')
    pause(1)
    
     U= UA+UR;
    figure(5); surf(U')
    title('Potencial Resultante')
    pause(1)    
    
      % Graficando la Repulsion 
    figure(6)
    X=1:50;
    Y=1:50;
    contour(X,-Y,-U,50);
    [Ux,Vy]=gradient(-U);
    hold on
    quiver(X,-Y,Ux,-Vy,1)
    hold off
    title('Gradiente y Contorno del Potencial')
    pause(1)
    cont=1;
    valor=1;
    menor=1000;
    while valor~=0
        for i=1:3
            for j=1:3
                if U(x1+i-2,y1+j-2)<menor
                    menor=U(x1+i-2,y1+j-2);
                    nx1=x1+i-2;
                    ny1=y1+j-2;
                end
                
                if U(x1+i-2,y1+j-2)==0
                    valor=0;
                end
                cont=cont+1;
            end
            figure(7);surf(M4'), view(90,90)
            title('Calculando Trayectoria')
            
            frame = getframe(gcf);
            writeVideo(writerObj, frame);
            
        end
        M4(nx1,ny1)=5;
        T(nx1,ny1)=1;
        x1=nx1;
        y1=ny1;
        
    end
close(writerObj);