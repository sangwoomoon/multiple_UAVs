function [X, Y, Z] = ned2xyz(refLat, refLong, refH, n, e, d) 

% Convert north,east,down coordinates (labeled n, e, d) to ECEF
% coordinates. The reference point (phi, lambda, h) must be given. 
% All distances are in meters

[Xr,Yr,Zr] = llh2xyz(refLat,refLong, refH); % location of reference point
phiP = atan2(Zr,sqrt(Xr^2+Yr^2)); % Geocentric latitude

X = -sin(phiP)*cos(refLong)*n -sin(refLong)*e  - cos(phiP)*cos(refLong)*d+ Xr;
Y = -sin(phiP)*sin(refLong)*n +cos(refLong)*e  - cos(phiP)*sin(refLong)*d+ Yr;
Z = cos(phiP)*n -sin(phiP)*d+ Zr;
% X = -sin(phiP)*cos(refLong)*n -sin(refLong)*e  - cos(refLong)*cos(phiP)*d + Xr;
% Y =  cos(refLong)*e - sin(refLong)*sin(phiP)*n + cos(phiP)*sin(refLong)*(-d) + Yr;
% Z = cos(phiP)*n + sin(phiP)*(-d) + Zr;
