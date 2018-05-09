function [P_next, V_next] = Motion_Planning(P,V,scan_pts,Cover_range,P_f,x_rate,y_rate,z_rate,linear_to_angular,dt,initial_velocity,minimum_height)

% BY USING 3RD METHOD

for iter_scan = 1 : length(scan_pts(:,1))
    normal(iter_scan,1:3) = P(1:3)-scan_pts(iter_scan,1:3);  % from scan_pts to location, NED coordinate
end

V_com(1:3) = zeros(1,3);
idx_cost = 0;
for iter_scan = 1 : length(scan_pts(:,1))
    if (-normal(iter_scan,1:3))*V(1:3)'/(norm(-normal(iter_scan,1:3))*norm(V(1:3))) > 0
        idx_cost = idx_cost + 1;
        normal_cost(idx_cost,1:3) = normal(iter_scan,1:3);
        cost(idx_cost) = (cos(acos(normal(iter_scan,1:3)*V(1:3)'/(norm(normal(iter_scan,1:3))*norm(V(1:3))))-pi)+1)*exp(-(norm(normal(iter_scan,1:3))-Cover_range));
    end
end

for iter_cost = 1 : idx_cost
    V_tmp = cost(iter_cost)*normal_cost(iter_cost,1:3)/norm(normal_cost(iter_cost,1:3));
    V_tmp_proj = V_tmp*V(1:3)'/(V(1:3)*V(1:3)')*V(1:3);
    V_com(1:3) = V_com(1:3) + V_tmp - V_tmp_proj;
end

V_direction(1:3) = initial_velocity*(P_f(1:3)-P(1:3))/norm((P_f(1:3)-P(1:3)));   %  NED coordinate

if (idx_cost == 0)
    V_com(1:3) = zeros(1,3);
end

% Linear Velocity Definition.
V_tmp = (1-exp(-norm(V_com(1:3))))*V_com(1:3) + exp(-norm(V_com(1:3)))*V_direction(1:3);  %  NED coordinate

% Velocity Bound.
if norm(V_tmp) > initial_velocity
    V_tmp = initial_velocity*V_tmp/norm(V_tmp);
end

% Linear Acceleration Bound.
mag_V = norm(V_tmp);

if (V_tmp(1,1)-V(1) < x_rate(1)*dt)
    V_tmp(1,1) = V(1) + x_rate(1)*dt;
elseif (V_tmp(1,1)-V(1) > x_rate(2)*dt)
    V_tmp(1,1) = V(1) + x_rate(2)*dt;
end

if (V_tmp(1,2)-V(2) < y_rate(1)*dt)
    V_tmp(1,2) = V(2) + y_rate(1)*dt;
elseif (V_tmp(1,2)-V(2) > y_rate(2)*dt)
    V_tmp(1,2) = V(2) + y_rate(2)*dt;
end

if (V_tmp(1,3)-V(3) < z_rate(1)*dt)
    V_tmp(1,3) = V(3) + z_rate(1)*dt;
elseif (V_tmp(1,3)-V(3) > z_rate(2)*dt)
    V_tmp(1,3) = V(3) + z_rate(2)*dt;
end

V_next(1:3) = mag_V*V_tmp/norm(V_tmp);
P_next(1:3) = P(1:3) + V_next(1:3)*dt;

% height fixing
P_next(3) = P(3);
V_next(3) = V(1);

% minimum height bound.
if P_next(3) > -minimum_height 
    V_next(3) = (-minimum_height-P(3))/dt; % NED Coordinate
    P_next(3) = -minimum_height;  % NED Coordinate
end

if (P_next(2)-P(2) < 0)
    V_next(4) = (-atan2(P_next(3)-P(3),-P_next(2)+P(2))-P(4))/(linear_to_angular(1)*dt);
else
    V_next(4) = (atan2(P_next(3)-P(3),P_next(2)-P(2))-P(4))/(linear_to_angular(1)*dt);
end

if (P_next(1)-P(1) < 0)
    V_next(5) = (atan2(P_next(3)-P(3),-P_next(1)+P(1))-P(5))/(linear_to_angular(2)*dt);
else
    V_next(5) = (-atan2(P_next(3)-P(3),P_next(1)-P(1))-P(5))/(linear_to_angular(2)*dt);
end

V_next(6) = (atan2(P_next(2)-P(2),P_next(1)-P(1))-P(6))/(linear_to_angular(3)*dt);
P_next(4:6) = P(4:6)+V_next(4:6)*dt;

