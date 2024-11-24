
clear all; close all; clc;
syms th1 th2 th3 L1 L2

Ry1=[cos(-th1) 0 sin(-th1) 0; 
      0 1 0 0;
     -sin(-th1) 0 cos(-th1) 0;
     0 0 0 1]

Rx1=[1 0 0 0; 0 cos(th2) -sin(th2) 0; 0 sin(th2) cos(th2) 0; 0 0 0 1  ]
Tz1=[1 0 0 0; 0 1 0 0 ; 0 0 1 -L1; 0 0 0 1]
Ry2=[cos(-th3) 0 sin(-th3) 0; 0 1 0 0;-sin(-th3) 0 cos(-th3) 0;0 0 0 1]
Tz2=[1 0 0 0; 0 1 0 0 ; 0 0 1 -L2; 0 0 0 1]

% [ cos(th1), -sin(th1)*sin(th2), -cos(th2)*sin(th1),  L1*sin(th1)]
% [        0,           cos(th2),          -sin(th2),            0]
% [ sin(th1),  cos(th1)*sin(th2),  cos(th1)*cos(th2), -L1*cos(th1)]
% [        0,                  0,                  0,            1]

M1= Ry1*Tz1*Rx1

M= Ry1*Rx1*Tz1*Ry2*Tz2 * [0 0 0 1]'

Mfail= [L2*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + L1*cos(th2)*sin(th1);
                                         L1*sin(th2) + L2*cos(th3)*sin(th2);
 L2*sin(th1)*sin(th3) - L2*cos(th1)*cos(th2)*cos(th3) + L1*cos(th1)*cos(th2);
                                                                          1]

th1=0; th2=0; th3=0 ; L1=1; L2=1
eval(M)
eval(Mfail)