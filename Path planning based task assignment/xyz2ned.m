function [N, E ,D]= xyz2ned(Xr, Yr, Zr, X, Y, Z) 
% convert ECEF coordinates to local north,east,down

phiP =  atan2(Zr,sqrt(Xr^2 + Yr^2)); 
lambda = atan2(Yr,Xr); 

N = -sin(phiP).*cos(lambda).*(X-Xr) - sin(phiP).*sin(lambda).*(Y-Yr) + cos(phiP).*(Z-Zr); 
E = -sin(lambda).*(X-Xr) + cos(lambda).*(Y-Yr); 
D = -(cos(phiP).*cos(lambda).*(X-Xr) + cos(phiP).*sin(lambda).*(Y-Yr) + sin(phiP).*(Z-Zr));

