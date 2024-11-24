function [xt_plus,x_minus,P_plus,P_minus,K]=filtro_kalman(psi,gamma,H,D,Q,R,S,xt_minus,P_minus,u,yruido)

%% ganancia de kalman 
K=P_minus*H'*inv(H*P_minus*H'+R);

%% correcion
xt_plus=xt_minus+K*(yruido-H*xt_minus-D*u);
P_plus=P_minus- K*H*P_minus;

%% prediccion
x_minus=(psi-S*inv(R)*H)*xt_plus + (gamma-S*inv(R)*D)*u + S*inv(R)*yruido;
P_minus=(psi-S*inv(R)*H)*P_plus*(psi-S*inv(R)*H)' + Q - S*inv(R)*S';
