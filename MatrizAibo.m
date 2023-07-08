clear all; close all
foot=46; tibia=100; thigh=100; hipy=50; hipz=85; neck=126.5; tzneck=67.9; txneck=53.9;
syms th1 th2 th3 th4 th5 th6 th7 th8

Tz1=[1 0 0 0;0 1 0 0; 0 0 1 foot; 0 0 0 1]
Rx1= [1 0 0 0; 0 cos(th1) -sin(th1) 0; 0 sin(th1) cos(th1) 0; 0 0 0 1]
Ry1= [cos(th2) 0 sin(th2) 0; 0 1 0 0; -sin(th2) 0 cos(th2) 0; 0 0 0 1]
Tz2=[1 0 0 0;0 1 0 0; 0 0 1 tibia; 0 0 0 1]
Ry2= [cos(th3) 0 sin(th3) 0; 0 1 0 0; -sin(th3) 0 cos(th3) 0; 0 0 0 1]
Tz3=[1 0 0 0;0 1 0 0; 0 0 1 thigh; 0 0 0 1]
Ry3= [cos(th4) 0 sin(th4) 0; 0 1 0 0; -sin(th4) 0 cos(th4) 0; 0 0 0 1]
Rx2= [1 0 0 0; 0 cos(th5) -sin(th5) 0; 0 sin(th5) cos(th5) 0; 0 0 0 1]

Rxn45= [1 0 0 0; 0 cos(-45*pi/180) -sin(-45*pi/180) 0; 0 sin(-45*pi/180) cos(-45*pi/180) 0; 0 0 0 1]
Ryy= [cos(th6) 0 sin(th6) 0; 0 1 0 0; -sin(th6) 0 cos(th6) 0; 0 0 0 1]
Rxp45= [1 0 0 0; 0 cos(45*pi/180) -sin(45*pi/180) 0; 0 sin(45*pi/180) cos(45*pi/180) 0; 0 0 0 1]

Ty1=[1 0 0 0;0 1 0 -hipy; 0 0 1 0; 0 0 0 1]
Tz4=[1 0 0 0;0 1 0 0; 0 0 1 hipz; 0 0 0 1]
Tz5=[1 0 0 0;0 1 0 0; 0 0 1 neck; 0 0 0 1]
Rz1= [cos(th7) -sin(th7) 0 0;sin(th7) cos(th7) 0 0 ;0 0 1 0; 0 0 0 1]
Ry4= [cos(th8) 0 sin(th8) 0; 0 1 0 0; -sin(th8) 0 cos(th8) 0; 0 0 0 1]
Tz6=[1 0 0 0;0 1 0 0; 0 0 1 tzneck; 0 0 0 1]
Tx1=[1 0 0 txneck;0 1 0 0; 0 0 1 0; 0 0 0 1]
 
% x=53.9 y=-50 z=525.9

M=Tz1*Rx1*Ry1*Tz2*Ry2*Tz3*Ry3*Rx2*Rxn45*Ryy*Rxp45*Ty1*Tz4*Tz5*Rz1*Ry4*Tz6*Tx1 *[0 0 0 1]'

th1=0; th2=0; th3=0; th4=0; th5=0; th6=0; th7=0; th8=0;

eval(M)
