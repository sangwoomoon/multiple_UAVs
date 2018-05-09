
%%%%%%%%%%%% REVISION NOTE %%%%%%%%%%%%%
%  2010.01.11   First dig-in,
%  2010.01.15   Method to find the Parameters for Plane was coded.
%  2010.01.18   Detection algorithm was coded.
%  2010.01.19   Method 1 was constructed.
%  2010.01.19   I got the serious problem about Method 1.
%  2010.01.20   2nd dig-in. Method 2 was constructed.
%  2010.01.21   I got the resonable result to use the Method 2 (proceedings on the 2010 IFAC ACA)
%  2010.01.25   some problems were detected on the result using Method 2.
%  2010.01.26   I found that those problems were caused by the coordinate problem (Matlab coordinate and NED coordinate are not same!)
%  2010.01.27   There was a problem for the iteration equation about the anglur velocity. (accerleration was required).
%               A problem was detected and detecting for ground was required.
%               Generated Path was not smooth.
%  2010.01.28   Critically Logical problem was detected in the code.
%               Algorithm was well defined after modification.
%  2010.01.30   Multi-agent Mission Planner was started to construct.
%  2010.02.01   A Critial Problem was found (initial velocity wasn't considered.)
%               Multi-agent Mission planning was well defined. However, a problem was detected because the ground wasn't considered.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Fundamental Elements Setting

clear all;
close all;
clc;

figure(1)
grid on
hold on
axis equal tight
axis([-100 100 -100 100 0 50]);

D2R = pi/180;
NUM_OBS = 7;
OBS_Height = [15 10 16 15 20 25 10];

Cover_range = 3;    % m
initial_velocity = 5; % m/s
CHK_admissible = 3;   % m

Mat2NED_xyz = [ 1 0 0; 0 -1 0; 0 0 -1];
Mat2NED_all = [Mat2NED_xyz zeros(3,3); zeros(3,3) Mat2NED_xyz];

dt = 0.2;
d_angle = 0.2;  %radian
d_angle_wpt = 1*D2R;

% scan spec.
range_theta = [70 110]; % down - up
range_psi = [30 150]; % left - right
Search_range = 20;

% heli. kenamatic spec.
x_rate = [-2 2];
y_rate = [-2 2];
z_rate = [-0.5 0.5];

linear_to_angular = [2 5 1];

roll_rate = [-50 50]*D2R;
pitch_rate = [-15 15]*D2R;
yaw_rate = [-60 60]*D2R;

[P_0xy(1),P_0xy(2)] = ginput(1);  % matlab coordinate
P = (Mat2NED_all*[P_0xy(1) P_0xy(2) 6.5 0 0 0]')';    %  NED coordinate
plot3(P(1),-P(2),-P(3),'bo');  % matlab coordinate

[P_fxy(1),P_fxy(2)] = ginput(1);  % matlab coordinate
P_f = (Mat2NED_all*[P_fxy(1) P_fxy(2) 10 0 0 0]')';    %  NED coordinate
plot3(P_f(1),-P_f(2),-P_f(3),'ro');  % matlab coordinate

[P_vxy(1),P_vxy(2)] = ginput(1);  % matlab coordinate
vector = [P(1),-P(2),-P(3);P_vxy(1),P_vxy(2),-P(3)];   % matlab coordinate
plot3(vector(:,1),vector(:,2),vector(:,3));   % matlab coordinate

% velocity assign (from scalar to vector)
V = (Mat2NED_all*(initial_velocity*[(P_vxy(1)-P_0xy(1)) (P_vxy(2)-P_0xy(2)) 0 0 0 0]/norm([(P_vxy(1)-P_0xy(1)) (P_vxy(2)-P_0xy(2)) 0]))')';
P(6) = atan2(-P_vxy(2)+P_0xy(2),P_vxy(1)-P_0xy(1));   % NED coordinate

%% Obstacle Setting %%

for iter_obs = 1 : NUM_OBS
    for iter_vert = 1 : 4
        [OBS_Vert(4*(iter_obs-1)+iter_vert,1), OBS_Vert(4*(iter_obs-1)+iter_vert,2)] = ginput(1);  % matlab coordinate
        OBS_Vert(4*(iter_obs-1)+iter_vert,2) = -OBS_Vert(4*(iter_obs-1)+iter_vert,2);  % NED coordinate
        plot(OBS_Vert(4*(iter_obs-1)+iter_vert,1), -OBS_Vert(4*(iter_obs-1)+iter_vert,2),'g.');
    end

    % 3 dimensional obstacle setting
    OBS_Vert_tmp = [OBS_Vert(4*(iter_obs-1)+1:4*iter_obs,:); OBS_Vert(4*(iter_obs-1)+1,:)]; % vertices setting
    clr = rand(1,3);
    OBS_Vert_lower = [OBS_Vert_tmp zeros(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate
    OBS_Vert_upper = [OBS_Vert_tmp -OBS_Height(iter_obs)*ones(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate

    patch(OBS_Vert_lower(:,1), -OBS_Vert_lower(:,2), -OBS_Vert_lower(:,3), clr);  % matlab coordinate
    patch(OBS_Vert_upper(:,1), -OBS_Vert_upper(:,2), -OBS_Vert_upper(:,3), clr);  % matlab coordinate
    
    % Define the Equations of Upper and Lower Planes
    Para_Plane(6*(iter_obs-1)+1,:) = null([OBS_Vert_lower(1:4,:) ones(4,1)])';  % NED coordinate
    Para_Plane(6*(iter_obs-1)+2,:) = null([OBS_Vert_upper(1:4,:) ones(4,1)])';  % NED coordinate

    for index = 1:length(OBS_Vert_tmp)-1
        temp_vert = [OBS_Vert_lower(index,:); OBS_Vert_lower(index+1,:); OBS_Vert_upper(index+1,:); OBS_Vert_upper(index,:)];  % NED coordinate
        Para_Plane(6*(iter_obs-1)+index+2,:) = null([temp_vert(1:4,:) ones(4,1)])';  % NED coordinate
        patch(temp_vert(:,1), -temp_vert(:,2), -temp_vert(:,3),clr);  % matlab coordinate
    end
end

%% DYNAMIC AND KINEMATIC EQUATIONS

X_u = -0.09865; Y_v = -0.2289; L_u = -0.2111; L_v = 0.1505; M_u = -0.08550; M_v = -0.05298;
N_p = -3.126; Z_w = -0.5024; Z_r = 0.9418; N_w = 0.07237; N_r = -2.742;

Z_col = 40.23; N_ped = 21.74; N_col = 2.303;

% X = [u v p q w r]'

A_dynamic = [X_u 0 0 0 0 0;
    0 Y_v 0 0 0 0;
    L_u L_v 0 0 0 0;
    M_u M_v 0 0 0 0;
    0 0 0 0 Z_w Z_r;
    0 0 N_p 0 N_w N_r];

B_dynamic = [0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 Z_col;
    0 0 N_ped N_col];

%% DATA ACQUATION And Find the new Waypoint.

det_prcd = 1; 
iter_prcd = 1;
idx = 1;

while (det_prcd)
    %% Euler Transformtion and Acquire the Scanned Range for Each Scanning Point
    idx_pt = 1;

    euler = [cos(P(iter_prcd,6))*cos(P(iter_prcd,5)) cos(P(iter_prcd,6))*sin(P(iter_prcd,4))*sin(P(iter_prcd,5))-cos(P(iter_prcd,4))*sin(P(iter_prcd,6)) cos(P(iter_prcd,4))*cos(P(iter_prcd,6))*sin(P(iter_prcd,5))+sin(P(iter_prcd,4))*sin(P(iter_prcd,6));
        sin(P(iter_prcd,6))*cos(P(iter_prcd,5)) sin(P(iter_prcd,6))*sin(P(iter_prcd,4))*sin(P(iter_prcd,5))+cos(P(iter_prcd,4))*cos(P(iter_prcd,6)) cos(P(iter_prcd,4))*sin(P(iter_prcd,6))*sin(P(iter_prcd,5))-sin(P(iter_prcd,4))*cos(P(iter_prcd,6));
        -sin(P(iter_prcd,5)) cos(P(iter_prcd,5))*sin(P(iter_prcd,4)) cos(P(iter_prcd,4))*cos(P(iter_prcd,5))];

    for theta = range_theta(1)*D2R : d_angle: range_theta(2)*D2R
        for psi = range_psi(1)*D2R : d_angle : range_psi(2)*D2R
            pt(idx_pt,1:3) = P(iter_prcd,1:3) + [Search_range*sin(theta)*sin(psi) -Search_range*sin(theta)*cos(psi) -Search_range*cos(theta)]*euler';  %  NED Coordinate
            idx_pt = idx_pt + 1;
        end
    end
    %% Obstacle Detection%%
    
    for iter_pt = 1 : idx_pt-1
        idx_detect_pts = 0;
        for iter_obs = 1 : NUM_OBS

            % Upper detection (all values are based on NED coordinate system)
            OBS_Vert_tmp = [OBS_Vert(4*(iter_obs-1)+1:4*iter_obs,:); OBS_Vert(4*(iter_obs-1)+1,:)]; % vertices setting
            OBS_Vert_lower = [OBS_Vert_tmp zeros(length(OBS_Vert_tmp(:,1)),1)];
            OBS_Vert_upper = [OBS_Vert_tmp -OBS_Height(iter_obs)*ones(length(OBS_Vert_tmp(:,1)),1)];
            t = (-Para_Plane(6*(iter_obs-1)+2,4)-P(iter_prcd,1:3)*(Para_Plane(6*(iter_obs-1)+2,1:3))')/((pt(iter_pt,1:3)-P(iter_prcd,1:3))*(Para_Plane(6*(iter_obs-1)+2,1:3))');

            if t > 0 && t < 1
                scan_pts_tmp = P(iter_prcd,1:3)+t*(pt(iter_pt,1:3)-P(iter_prcd,1:3));
                if (inpolygon(scan_pts_tmp(1,1),scan_pts_tmp(1,2),OBS_Vert_upper(:,1),OBS_Vert_upper(:,2)))
                    idx_detect_pts = idx_detect_pts + 1;
                    detect_pts_tmp(idx_detect_pts,:) = scan_pts_tmp;
                end
            end

            % Side detection
            for index = 1:length(OBS_Vert_tmp)-1
                temp_vert = [OBS_Vert_lower(index,:); OBS_Vert_lower(index+1,:); OBS_Vert_upper(index+1,:); OBS_Vert_upper(index,:)];
                t = (-Para_Plane(6*(iter_obs-1)+index+2,4)-P(iter_prcd,1:3)*(Para_Plane(6*(iter_obs-1)+index+2,1:3))')/((pt(iter_pt,1:3)-P(iter_prcd,1:3))*(Para_Plane(6*(iter_obs-1)+index+2,1:3))');

                if t > 0 && t < 1
                    scan_pts_tmp = P(iter_prcd,1:3)+t*(pt(iter_pt,1:3)-P(iter_prcd,1:3));
                    if (inpolygon(scan_pts_tmp(1,1),scan_pts_tmp(1,3),temp_vert(:,1),temp_vert(:,3))) && (inpolygon(scan_pts_tmp(1,2),scan_pts_tmp(1,3),temp_vert(:,2),temp_vert(:,3)))
                        idx_detect_pts = idx_detect_pts + 1;
                        detect_pts_tmp(idx_detect_pts,:) = scan_pts_tmp;
                    end
                end
            end
        end

        if (idx_detect_pts == 1)
            scan_pts(idx,:)= detect_pts_tmp;
            idx = idx + 1;
        elseif (idx_detect_pts > 1)
            scan_pts(idx,:) = detect_pts_tmp(1,:);
            for iter_scan = 2 : idx_detect_pts
                if distance(P(iter_prcd,1:3),scan_pts(idx,:)) > distance(P(iter_prcd,1:3),detect_pts_tmp(idx_detect_pts,:))
                    scan_pts(idx,:) = detect_pts_tmp(idx_detect_pts,:);  % NED coordinate
                end
            end
            idx = idx + 1;
        end
        clear detect_pts_tmp;
    end
    
    %% METHOD # 3
    
    for iter_scan = 1 : idx-1
        normal(iter_scan,1:3) = P(iter_prcd,1:3)-scan_pts(iter_scan,1:3);  % from scan_pts to location, NED coordinate
    end
    
    V_com(iter_prcd,1:3) = zeros(1,3);
    idx_cost = 0;
    for iter_scan = 1 : idx-1
        if (-normal(iter_scan,1:3))*V(iter_prcd,1:3)'/(norm(-normal(iter_scan,1:3))*norm(V(iter_prcd,1:3))) > 0
            idx_cost = idx_cost + 1;
            normal_cost(idx_cost,1:3) = normal(iter_scan,1:3);
            cost(idx_cost) = (cos(acos(normal(iter_scan,1:3)*V(iter_prcd,1:3)'/(norm(normal(iter_scan,1:3))*norm(V(iter_prcd,1:3))))-pi)+1)*exp(-(norm(normal(iter_scan,1:3))-Cover_range));
        end
    end
    
    for iter_cost = 1 : idx_cost
        V_tmp = cost(iter_cost)*normal_cost(iter_cost,1:3)/norm(normal_cost(iter_cost,1:3));
        V_tmp_proj = V_tmp*V(iter_prcd,1:3)'/(V(iter_prcd,1:3)*V(iter_prcd,1:3)')*V(iter_prcd,1:3);
        V_com(iter_prcd,1:3) = V_com(iter_prcd,1:3) + V_tmp - V_tmp_proj;
    end
    
    V_direction(iter_prcd,1:3) = norm(V(1,1:3))*(P_f(1,1:3)-P(iter_prcd,1:3))/norm((P_f(1,1:3)-P(iter_prcd,1:3)));   %  NED coordinate
    
    if (idx_cost == 0)
        V_com(iter_prcd,1:3) = zeros(1,3);
    end
    
    % Linear Velocity Bound.
    V_tmp = (1-exp(-norm(V_com(iter_prcd,1:3))))*V_com(iter_prcd,1:3) + exp(-norm(V_com(iter_prcd,1:3)))*V_direction(iter_prcd,1:3);  %  NED coordinate
    if norm(V_tmp) > norm(V(1,1:3))
        V_tmp = norm(V(1,1:3))*V_tmp/norm(V_tmp);
    end
    
    % Linear Acceleration Bound.
    mag_V = norm(V_tmp);
    
    if (V_tmp(1,1)-V(iter_prcd,1) < x_rate(1)*dt)
        V_tmp(1,1) = V(iter_prcd,1) + x_rate(1)*dt;
    elseif (V_tmp(1,1)-V(iter_prcd,1) > x_rate(2)*dt)
        V_tmp(1,1) = V(iter_prcd,1) + x_rate(2)*dt;
    end

    if (V_tmp(1,2)-V(iter_prcd,2) < y_rate(1)*dt)
        V_tmp(1,2) = V(iter_prcd,2) + y_rate(1)*dt;
    elseif (V_tmp(1,2)-V(iter_prcd,2) > y_rate(2)*dt)
        V_tmp(1,2) = V(iter_prcd,2) + y_rate(2)*dt;
    end

    if (V_tmp(1,3)-V(iter_prcd,3) < z_rate(1)*dt)
        V_tmp(1,3) = V(iter_prcd,3) + z_rate(1)*dt;
    elseif (V_tmp(1,3)-V(iter_prcd,3) > z_rate(2)*dt)
        V_tmp(1,3) = V(iter_prcd,3) + z_rate(2)*dt;
    end
    
    V(iter_prcd+1,1:3) = mag_V*V_tmp/norm(V_tmp);
    P(iter_prcd+1,1:3) = P(iter_prcd,1:3) + V(iter_prcd+1,1:3)*dt;
    
    if (P(iter_prcd+1,2)-P(iter_prcd,2) < 0) 
        V(iter_prcd+1,4) = (-atan2(P(iter_prcd+1,3)-P(iter_prcd,3),-P(iter_prcd+1,2)+P(iter_prcd,2))-P(iter_prcd,4))/(linear_to_angular(1)*dt);
    else
        V(iter_prcd+1,4) = (atan2(P(iter_prcd+1,3)-P(iter_prcd,3),P(iter_prcd+1,2)-P(iter_prcd,2))-P(iter_prcd,4))/(linear_to_angular(1)*dt);
    end
          
    if (P(iter_prcd+1,1)-P(iter_prcd,1) < 0)
        V(iter_prcd+1,5) = (atan2(P(iter_prcd+1,3)-P(iter_prcd,3),-P(iter_prcd+1,1)+P(iter_prcd,1))-P(iter_prcd,5))/(linear_to_angular(2)*dt);
    else
        V(iter_prcd+1,5) = (-atan2(P(iter_prcd+1,3)-P(iter_prcd,3),P(iter_prcd+1,1)-P(iter_prcd,1))-P(iter_prcd,5))/(linear_to_angular(2)*dt);
    end
       
    V(iter_prcd+1,6) = (atan2(P(iter_prcd+1,2)-P(iter_prcd,2),P(iter_prcd+1,1)-P(iter_prcd,1))-P(iter_prcd,6))/(linear_to_angular(3)*dt);    
    P(iter_prcd+1,4:6) = P(iter_prcd,4:6)+V(iter_prcd+1,4:6)*dt;
    clear normal normal_cost;
    if (distance(P(iter_prcd+1,1:3),P_f(1:3)) < CHK_admissible)
        det_prcd = 0;
    elseif (iter_prcd == 200)
        det_prcd = 0;
    else
        iter_prcd = iter_prcd + 1;
    end
    
%     figure(2)
%     grid on, axis equal; hold on;
%     if (idx > 1)
%         plot3(scan_pts(:,1),-scan_pts(:,2),-scan_pts(:,3),'r.');
%     end
% 
%     plot3(P(iter_prcd-1:iter_prcd,1),-P(iter_prcd-1:iter_prcd,2),-P(iter_prcd-1:iter_prcd,3),'color','blue');
    
end


figure(2)
grid on, axis equal; hold on;
plot3(scan_pts(:,1),-scan_pts(:,2),-scan_pts(:,3),'r.');
plot3(P(:,1),-P(:,2),-P(:,3),'b-');

figure(1)
plot3(P(:,1),-P(:,2),-P(:,3),'r-');

figure(3)
x = 0 : 0.2 : 0.2*iter_prcd;
for i = 1 : iter_prcd+1
    V_mag(i) = norm(V(i,1:3));
end
plot(x,V_mag,x,V(:,1),x,V(:,2),x,V(:,3));

figure(4)
plot(x,P(:,4)/D2R,x,P(:,5)/D2R,x,P(:,6)/D2R);



%% Determine the Cost Function and The New Waypoint (archive)

    %% METHOD # 1 (IS NOT APPLICABLE).
    
%     idx_wpt = 0;
%     V_tmp = zeros(1,6);
%     for theta = spec_theta(1)*D2R : d_angle_wpt: spec_theta(2)*D2R
%         for psi = spec_psi(1)*D2R : d_angle_wpt : spec_psi(2)*D2R
%             idx_wpt = idx_wpt + 1;
%             V_tmp(idx_wpt,:) = [V_const*sin(theta)*sin(psi) V_const*sin(theta)*cos(psi) V_const*cos(theta)...
%                 (roll_rate(2)-roll_rate(1))/(spec_psi(2)-spec_psi(1))*(psi-90*D2R) (pitch_rate(2)-pitch_rate(1))/(spec_theta(2)-spec_theta(1))*(theta-90*D2R) (psi-P(iter_prcd,6))/dt];
%             WPT_tmp(idx_wpt,1:3) = P(iter_prcd,1:3) + V_tmp(idx_wpt,1:3)*dt*euler';
%             WPT_tmp(idx_wpt,4:6) = P(iter_prcd,4:6) + V_tmp(idx_wpt, 4:6)*dt;
%         end
%     end
% 
%     cost = exp(-distance(WPT_tmp(1,1:3),scan_pts(1,1:3)));
%     for iter_scan = 2 : idx-1
%         if cost < exp(-distance(WPT_tmp(1,1:3),scan_pts(iter_scan,1:3)))
%             cost = exp(-distance(WPT_tmp(1,1:3),scan_pts(iter_scan,1:3)));
%         end
%     end
%     P(iter_prcd+1,:) = WPT_tmp(1,:);
%     cost_test(1) = cost;
%     
%     for iter_wpt = 2 : idx_wpt
%         cost_tmp = exp(-distance(WPT_tmp(iter_wpt,1:3),scan_pts(1,1:3)));
%         for iter_scan = 2 : idx-1
%             if cost_tmp < exp(-distance(WPT_tmp(iter_wpt,1:3),scan_pts(iter_scan,1:3)))
%                 cost_tmp = exp(-distance(WPT_tmp(iter_wpt,1:3),scan_pts(iter_scan,1:3)));
%             end
%         end
%         
%         cost_test(iter_wpt) = cost_tmp;
%         
%         if(cost_tmp < cost)
%             cost = cost_tmp;
%             P(iter_prcd+1,:) = WPT_tmp(iter_wpt,:);
%         end
%     end

    %% METHOD # 2

%     for iter_scan = 1 : idx-1
%         normal(iter_scan,1:3) = P(iter_prcd,1:3)-scan_pts(iter_scan,1:3);  % from scan_pts to location
%     end
%     
%     K = 1;
%     W_com = 0.1;
%     W_com_2 = 0.5;
%     V_com = zeros(1,3);
%     idx_cost = 0;
%     clr = rand(1,3);
%     for iter_scan = 1 : idx-1
%         if ((-normal(iter_scan,1:3)))*V(iter_prcd,1:3)'/(norm(-normal(iter_scan,1:3))*norm(V(iter_prcd,1:3))) > 0
%             idx_cost = idx_cost + 1;
%             cost(idx_cost) = K*(cos(acos(normal(iter_scan,1:3)*V(iter_prcd,1:3)'/(norm(normal(iter_scan,1:3))*norm(V(iter_prcd,1:3))))-pi)+1)*exp(-(norm(normal(iter_scan,1:3)))^2);
%             plot3(scan_pts(iter_scan,1),scan_pts(iter_scan,2),scan_pts(iter_scan,3),'color',clr,'linestyle','.'); hold on;
%         end
%     end
%     
%     for iter_cost = 1 : idx_cost
%         V_tmp = cost(iter_cost)*normal(iter_cost,1:3)/norm(normal(iter_cost,1:3));
%         V_tmp_proj = V_tmp*V(iter_prcd,1:3)'/(V(iter_prcd,1:3)*V(iter_prcd,1:3)')*V(iter_prcd,1:3);
%         B(iter_cost) = 1-exp(-cost(iter_cost));
%         V_com = V_com + V_tmp - B(iter_cost)*(V_tmp_proj - W_com_2*V(iter_prcd,1:3));
%     end
%     
%     V_direction(iter_prcd,1:6) = [norm(V(iter_prcd,1:3))*(P_f(1,1:3)-P(iter_prcd,1:3))/norm((P_f(1,1:3)-P(iter_prcd,1:3))),zeros(1,3)];
%     
%     % Velocity Bound.
%     V_tmp = W_com*V_com(1:3) + V_direction(iter_prcd,1:3);
%     if norm(V_tmp) > norm(V(1,1:3))
%         V_tmp = norm(V(1,1:3))*V_tmp/norm(V_tmp);
%     end
%     V(iter_prcd+1,1:3) = V_tmp;
%     P(iter_prcd+1,1:3) = P(iter_prcd,1:3) + V(iter_prcd+1,1:3)*dt;
%     V(iter_prcd+1,4) = (-atan2(P(iter_prcd+1,3)-P(iter_prcd,3),P(iter_prcd+1,2)-P(iter_prcd,2))-P(iter_prcd,4))/dt;
%     V(iter_prcd+1,5) = (-atan2(P(iter_prcd+1,3)-P(iter_prcd,3),P(iter_prcd+1,1)-P(iter_prcd,1))-P(iter_prcd,5))/dt;
%     V(iter_prcd+1,6) = (atan2(P(iter_prcd+1,2)-P(iter_prcd,2),P(iter_prcd+1,1)-P(iter_prcd,1))-P(iter_prcd,6))/dt;
%     P(iter_prcd+1,4:6) = P(iter_prcd,4:6)+V(iter_prcd+1,4:6)*dt;