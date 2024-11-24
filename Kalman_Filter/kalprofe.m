% Robotica Probabilistica II/2023
% Marcelo Saavedra Alcoba
clc; clear all; close all

psi=[0.9 0 0 ; 0 0.3 0; 0 0 0.01]
gamma=ones(3,1)
H=[0.8 0.1 -0.01]
D=0

%%
[fpsi cpsi]=size(psi)
[fgam cgam]=size(gamma)
[fh ch]=size(H)
[fd cd]=size(D)

%% Condiciones iniciales
P0_plus=eye(fpsi)   % P0_plus=50*eye(fpsi) 
x0=mvnrnd(ones(fpsi,1),P0_plus)' %x0=mvnrnd(100*ones(fpsi,1),P0_plus)'

[fy,cy]=size(H*x0)
alfa=0.5; Q=alfa*eye(fpsi,fpsi)  % variar alfa 0.01 - 2.5
beta=1.0; R=beta*eye(fy,fy)
eta=0.1; S=eta*eye(fpsi,fy)

%% Simular salida de control
T=80;
t=1:T/100:T;
u=sin(50*t);
figure
plot(t,u)
title('Senial de entrada')


  
  xtREAL=[0 0 0]'
  xtRUIDO=[0 0 0]'
  xESTIMADO=[0 0 0]'
  
  kVector=[0 0 0]';
  sigmVector12=0
  sigmVector21=0
%% Estimador de estados
for i=1:T
  wt=mvnrnd(zeros(fpsi,1),Q)';
  vt=mvnrnd(zeros(fy,1),R)';
  
  if i==1
    ytruido=H*x0 + D*u(i)+vt;
    yreal=H*x0 + D*u(i);
    P_minus=P0_plus;
    xstr1=x0;
    xtruido=psi*x0+gamma*u(i)+wt;
    xtreal=psi*x0+gamma*u(i);
  else
    xtruido=psi*xtreal+gamma*u(i)+wt;
    ytruido=H*xtreal+D*u(i)+vt;
    xtreal=psi*xtreal+gamma*u(i); 
  end    
    
  [xst,xstr1,P_plus,P_minus,K]=filtro_kalman(psi,gamma,H,D,Q,R,S,xstr1,P_minus,u(i),ytruido);

  xtREAL=[xtREAL xtreal]
  xtRUIDO=[xtRUIDO xtruido]
  xESTIMADO=[xESTIMADO xst]
  
  kVector=[kVector K];
  
  sigmVector12=[sigmVector12 P_plus(1,2)];
  sigmVector21=[sigmVector21 P_plus(2,1)];
  
end    

for j=1:3
    graficar(j,xtRUIDO(j,:),xtREAL(j,:),xESTIMADO(j,:))
end


