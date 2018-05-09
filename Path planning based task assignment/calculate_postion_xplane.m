function [wpt,flag_change] = calculate_postion_xplane(wpt_prev,WPT,HLI_vel,HLI_max_dyaw,dt,chk_range)

% WPT :: waypoints solved by using line-line search. in this function, this
% point can be considered as the goal point.

theta = atan2(WPT(1,2)-wpt_prev(2),WPT(1,1)-wpt_prev(1));
if abs(theta-wpt_prev(3)) > HLI_max_dyaw*dt
    if (theta-wpt_prev(3) > HLI_max_dyaw*dt)
        theta = HLI_max_dyaw*dt+wpt_prev(3);
    elseif (theta-wpt_prev(3) < -HLI_max_dyaw*dt)
        theta = -HLI_max_dyaw*dt+wpt_prev(3);
    end
end

% position defined by X_plane simulator 
% first, change the scale for syncronization
% wpt_data_xplane = get(); 

wpt = [wpt_prev(1)+HLI_vel*dt*cos(theta),wpt_prev(2)+HLI_vel*dt*sin(theta),theta];


if distance(wpt(1:2),WPT) < chk_range
    flag_change = 1;
else
    flag_change = 0;
end
