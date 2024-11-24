% Robotica Probabilistica II/2023
% EKF tarea 2
% Marcelo Saavedra Alcoba

%% Parte sistema lineal del proceso
clc; clear all; close all
A=diag([.95 0.9]);
B=zeros(2,1);
C=zeros(1,2);
D=0;


[fa ca]=size(A);
[fb cb]=size(B);
[fc cc]=size(C);
[fd cd]=size(D);

%% A. Parte del Sistema No Lineal
syms t x1 x2;
g = (cos((t/100)+x1))*x2;
%% Definicion del modelo linealizado
symg = symvar(g);
NymEstadosg = length(symg);
symbolog = symg(2:NymEstadosg);

Ht = jacobian(g,symbolog);

%% Condiciones Iniciales
[fa,ca]=size(A);

Sigma0= 0.1 * eye(fa);
Sigmaest0=0.1 * eye(fa);

x0 = mvnrnd (15*ones(fa,1),Sigma0)';
xst0=mvnrnd (15*ones(fa,1),Sigmaest0)';

[fy,cy]=size(C*x0); % Dimension de la salida
Q= diag([.1 .5]); % covarianza ruido del proceso
R = [0.01];
S = [0;0];

%%
xtRUIDO=zeros(fa,1);
xtREAL=zeros(fa,1);
xESTIMADO=zeros(fa,1);
kVector=zeros(fa,1);
sigmVector12=0;
sigmVector21=0;
ytRUIDO=0;


T=120;

%%
for i=0:T-1
    wt=mvnrnd(zeros(fa,1),Q)';
    vt=mvnrnd(zeros(fy,1),R)';
    t = i;
    if i==0
        xst1= xst0;
        Sigma1 = Sigma0;
        
        yruido = double(subs(g,symg,[t,x0'])) + vt;
        xtruido = A*x0 + wt;
        
        yreal = double(subs(g,symg,[t,x0']));
        xtreal= x0;
 
    else
        yruido = double(subs(g,symg,[t,xtreal'])) + vt;
        xtruido = A*xtruido + wt;
        
        yreal = double(subs(g,symg,[t,xtreal']));
        xtreal= A * xtreal;
    end
    
    [xst,xst1,Sigma,Sigma1,K]=...
        kalmanExt(t,g,A,Ht,symg,Q,R,S,xst1,Sigma1,yruido);
    xtREAL=[xtREAL xtreal];
    xESTIMADO=[xESTIMADO xst];
    kVector=[kVector K];
  


figure(1)
subplot(211)
plot(xtREAL(1,:),'r')
grid on
hold on
plot(xESTIMADO(1,:),'b')
title('EKF - GRAFICA ESTADO REAL-ESTADO ESTIMADO 1');
legend('a0 Real','a0 Estimado')
xlabel('Iteraciones')
ylabel('a0')

subplot(212)
plot(xtREAL(2,:),'r')
grid on
hold on
plot(xESTIMADO(2,:),'b')
title('EKF - GRAFICA ESTADO REAL-ESTADO ESTIMADO 2');
legend('b0 Real','b0 Estimado')
xlabel('Iteraciones')
ylabel('b0')


end    

