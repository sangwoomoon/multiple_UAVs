 function [phi, lambda, h] = xyz2llh(X,Y,Z) 

 a = 6378137.0; % earth semimajor axis in meters
 f = 1/298.257223563; % reciprocal flattening
 b = a*(1-f);% semi-minor axis
 e2 = 2*f-f^2;% first eccentricity squared
 ep2 = f*(2-f)/((1-f)^2); % second eccentricity squared
 
 r2 = X.^2+Y.^2;
 r = sqrt(r2);
 E2 = a^2 - b^2;
 F = 54*b^2*Z.^2;
 G = r2 + (1-e2)*Z.^2 - e2*E2;
 c = (e2*e2*F.*r2)./(G.*G.*G);
 s = ( 1 + c + sqrt(c.*c + 2*c) ).^(1/3);
 P = F./(3*(s+1./s+1).^2.*G.*G);
 Q = sqrt(1+2*e2*e2*P);
 ro = -(e2*P.*r)./(1+Q) + sqrt((a*a/2)*(1+1./Q) - ((1-e2)*P.*Z.^2)./(Q.*(1+Q)) - P.*r2/2);
 tmp = (r - e2*ro).^2;
 U = sqrt( tmp + Z.^2 );
 V = sqrt( tmp + (1-e2)*Z.^2 );
 zo = (b^2*Z)./(a*V);
 
 
 h = U.*( 1 - b^2./(a*V));
 phi = atan( (Z + ep2*zo)./r ); % lat [rad]
 lambda = atan2(Y,X);           % long[rad]
