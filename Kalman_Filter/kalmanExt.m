function [xst, xst1, Sigma, Sigma1, K]=kalmanExt(t,g,A,Ht,symg,Q,R,S,xst,Sigma,yruido)
%% ganacia Kalman
C = double(subs(Ht,symg,[t,xst']));
K=Sigma*C'*inv(C*Sigma*C'+R);

%% Measurement update
xst = xst + K*(yruido - double(subs(g,symg,[t,xst']))); 
Sigma = Sigma - K*C*Sigma;

%% time update
xst1 = (A - S* inv(R)*C)*xst + S* inv(R)*yruido; 
Sigma1 = Q-S*inv(R)*S' + (A - S* inv(R)*C)*Sigma*(A - S* inv(R)*C)';