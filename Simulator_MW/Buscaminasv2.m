clc;
clear all;
close all;

hx=-9.5;
hy=-9.5;
phi=0;

fig=figure;
set(fig,'position',[10 60 980 600]);
axis('image');

axis([-10 10 -10 10 -0.1 3]);
grid on;

MobileRobot;
M1=MobilePlot2(hx, hy, phi);
hold on
%plot(hx, hy, 'Or');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
camlight('right');
grid on;
%%%%%%%%%%%%%%%%%%% Posicion %%%%%%%%%%
%%

ts=0.1;
tf=1;
tt='a';
dire='e';
band=0;
while tt~=0
   tt=input('opcion: ')
   % dir este 
   if tt==8 && dire=='e'
      hx=hx+1; 
      tt=10;
   end
   if tt==2 && dire=='e'
      hx=hx-1;
      tt=10;
   end 
   if tt==4 && dire=='e'
      phi=phi+pi/2; 
      dire='n';
      tt=10;
   end
   if tt==6 && dire=='e'
       phi=phi-pi/2; 
       dire='s';
       tt=10;
    end  
   % dir norte
   if tt==8 && dire=='n'
      hy=hy+1;
      tt=10;
   end
   if tt==2 && dire=='n'
      hy=hy-1;
      tt=10;
   end 
   if tt==6 && dire=='n'
      phi=0; 
      dire='e';
      tt=10;
   end 
   if tt==4 && dire=='n'
      phi=phi+pi/2; 
      dire='o';
      tt=10;
   end   
   % dir oeste
   if tt==8 && dire=='o'
      hx=hx-1; 
      tt=10;
   end   
   if tt==2 && dire=='o'
      hx=hx+1; 
      tt=10;
   end  
   if tt==4 && dire=='o'
      phi=phi+pi/2;
      dire='s';
      tt=10;
   end   
   if tt==6 && dire=='o'
      phi=phi-pi/2;
      dire='n';
      tt=10;
   end  
   % dir sur
    if tt==8 && dire=='s'
      hy=hy-1; 
      tt=10;
   end   
   if tt==2 && dire=='s'
      hy=hy+1; 
      tt=10;
   end 
   if tt==4 && dire=='s'
      phi=0;
      dire='e';
      tt=10;
   end   
   if tt==6 && dire=='s'
      phi=phi-pi/2;
      dire='o';
      tt=10;
   end     
   
   
   %Bomba!!!
    if tt==5
       plot(hx, hy, '*r');
    end
   
   
    delete (M1)
    M1=MobilePlot2(hx, hy, phi);
end










