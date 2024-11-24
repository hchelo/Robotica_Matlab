% ETAPA DE MEASURMENT UPDATE: CORRECCIÓN
% Por cada caracteristica:
function [Kt,pose,Sigma] = measurement_update(H,Sigma,R,pose,zt,zt_est)
%%
Kt = Sigma*H'*inv(H*Sigma*H'+R);
pose = pose+Kt*(zt-zt_est);
Sigma = Sigma-Kt*H*Sigma;

