function [X,Y,Z] = llh2xyz(lat,long,h) 
% Convert lat, long, height in WGS84 to ECEF X,Y,Z
% lat and long given in radian
% altitude should be given in meters


a = 6378137.0; % earth semimajor axis in meters 
f = 1/298.257223563; % reciprocal flattening 
e2 = 2*f -f^2; % eccentricity squared 

chi = sqrt(1-e2*(sin(lat)).^2); 
X = (a./chi +h).*cos(lat).*cos(long); 
Y = (a./chi +h).*cos(lat).*sin(long); 
Z = (a*(1-e2)./chi + h).*sin(lat);
  
