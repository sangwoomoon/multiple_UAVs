function [P_next, V_next] = Motion_Planning_2(P_now,V_now,idx_UAV,NUM_UAV,scan_pts,Cover_range,P_f,dt,initial_velocity,D_desired,VELO_UAV_MAX,VELO_UAV_RATE,MINIMUM_HEIGHT,GROUP_NUM,MASTER_SLAVE,POS_desired)

% BY USING 4TH METHOD.
% ASSUMPTION : RUAVs so that rolling and pitching angle are zero.

% Euler Transformtion to get the PROCEEDING POINT.
D2R = pi/180;

if (MASTER_SLAVE(idx_UAV) == 0)  % if slave,
    for iter_UAV = 1 : NUM_UAV
        if (GROUP_NUM(iter_UAV) == GROUP_NUM(idx_UAV)) && (MASTER_SLAVE(iter_UAV) == 1)  % find the master in that group.
            euler = Calculate_Euler(P_now(6*(iter_UAV-1)+4:6*(iter_UAV-1)+6));
            P_f = P_now(6*(iter_UAV-1)+1:6*(iter_UAV-1)+3)+D_desired*POS_desired/norm(POS_desired)*euler';       % NED Coordinate, assign a dynamic final point for a slave
            FINAL_DIST = distance(P_f,P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3));
        end
    end
        
    velocity = initial_velocity*(1+(FINAL_DIST-D_desired)/D_desired);  % velocity magnitude formula
    
    % velocity profile definition (saturation for magnitude and rate)
    if (velocity - norm(V_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)) > VELO_UAV_RATE)
        velocity = norm(V_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)) + VELO_UAV_RATE;
    elseif (velocity - norm(V_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)) < -VELO_UAV_RATE)
        velocity = norm(V_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)) - VELO_UAV_RATE;
    end
    
    if (velocity > VELO_UAV_MAX)
        velocity = VELO_UAV_MAX;
    end
    
    prcd_range = velocity*dt;
else  % if master (assume that master has a constant speed)
    prcd_range = initial_velocity*dt;
end

diff_angle = 5;
range_theta = [80 100];
range_psi = [80 100];
diff_gap_theta = (range_theta(2)-range_theta(1))/diff_angle+1;
diff_gap_psi = (range_psi(2)-range_psi(1))/diff_angle+1;

% WEIGHTING FACTORS for cost function.
c_1 = 1;
c_2 = 0.2;
c_3 = 0.1;

idx_pt = 0;

euler = Calculate_Euler(P_now(6*(idx_UAV-1)+4:6*(idx_UAV-1)+6));

for theta = range_theta(1)*D2R : diff_angle*D2R : range_theta(2)*D2R
    for psi = range_psi(1)*D2R : diff_angle*D2R : range_psi(2)*D2R
        idx_pt = idx_pt + 1;        
        pt(idx_pt,1:3) = P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3) + prcd_range*[sin(theta)*sin(psi) -sin(theta)*cos(psi) -cos(theta)]*euler';  %  NED Coordinate
    end
end

% 1 STEP BASED OPTIMIZATION.

for iter_idx_pt = 1 : idx_pt

    % Calculate Velocity Vector
    V(1:3) = (pt(iter_idx_pt,1:3)-P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3))/dt;
    
    % COST FOR GOAL POINT
    cost_goal = (1-exp(-norm(P_f(1:3)-pt(iter_idx_pt,1:3))/norm(P_f(1:3)-P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3))));

    % COST FOR OBSTACLES
    idx_cost = 0;
    cost_obs = 0;
    if (length(scan_pts(:,1)) > 1)
        for iter_scan = 1 : length(scan_pts(:,1))
            normal_obs = pt(iter_idx_pt,1:3)-scan_pts(iter_scan,1:3);  % from scan_pts to location, NED coordinate
            normal_obs_den = P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)-scan_pts(iter_scan,1:3);
            cost_obs(iter_scan) = ...
                (0.5*cos(acos(normal_obs*V(1:3)'/(norm(normal_obs)*norm(V(1:3))))-pi)+0.5)*exp(-(norm(normal_obs)-Cover_range)/norm(normal_obs_den));
        end
    end

    
    % COST FOR AGENTS
    idx_cost = 0;
    cost_agent = 0;
    if (NUM_UAV > 1)   % for multiple UAVs
        for iter_UAV = 1 : NUM_UAV
            if (iter_UAV ~= idx_UAV)
                normal_agent = pt(iter_idx_pt,1:3)-P_now(6*(iter_UAV-1)+1:6*(iter_UAV-1)+3);
                normal_agent_den = P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)-P_now(6*(iter_UAV-1)+1:6*(iter_UAV-1)+3);
                cost_agent = cost_agent+ ...
                    (0.5*cos(acos(normal_agent*V_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)'/(norm(normal_agent)*norm(V_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3))))-pi)+0.5)*...
                    exp(-(norm(normal_agent)-Cover_range)/norm(normal_agent_den)^2);
                    % V_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3)
            end
        end
    end

    % FINAL COST FUNCTION
    %  J = fn(goal point, obstacle avoidance, other agents collision avoidance).
    if (idx_UAV ~= 0)
        cost_res(iter_idx_pt) = c_1*cost_goal + c_2*(max(cost_obs)) + c_3*(max(cost_agent));
    else
        cost_res(iter_idx_pt) = c_1*cost_goal;
    end
    if (pt(iter_idx_pt,3) > - MINIMUM_HEIGHT)  % NED Coordinate
        cost_res(iter_idx_pt) = 999; %% impossible to go
    end
    
    if (iter_idx_pt == 1)
        P_next = [pt(1,1:3) 0 0  atan2(pt(1,2)-P_now(6*(idx_UAV-1)+2),pt(1,1)-P_now(6*(idx_UAV-1)+1))];
        V_next = [(P_next(1:3)-P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3))/dt 0 0 (P_next(6)-P_now(6*(idx_UAV-1)+6))/dt];
        cost_min = cost_res(1);
        idx_min = 1;
    else
        if (cost_res(iter_idx_pt) < cost_min)
            P_next = [pt(iter_idx_pt,1:3) 0 0  atan2(pt(iter_idx_pt,2)-P_now(6*(idx_UAV-1)+2),pt(iter_idx_pt,1)-P_now(6*(idx_UAV-1)+1))];
            V_next = [(P_next(1:3)-P_now(6*(idx_UAV-1)+1:6*(idx_UAV-1)+3))/dt 0 0 (P_next(6)-P_now(6*(idx_UAV-1)+6))/dt];
            cost_min = cost_res(iter_idx_pt);
            idx_min = iter_idx_pt;
        end
    end
 
    clear cost_obs cost_agent normal_obs normal_obs_den normal_agent normal_agent_den;
end


% TEST PLOTTING FOR COST FUNCTION

% plot3(pt(:,1),-pt(:,2),-pt(:,3),'g.');
% plot3(P_next(1),-P_next(2),-P_next(3),'k.');

% if (MASTER_SLAVE(idx_UAV) == 0)
%     figure(1)
%     plot3(P_f(1),-P_f(2),-P_f(3),'r.');
% end

% for iter_x = 1 : diff_gap_theta
%     for iter_y = 1 : diff_gap_psi
%         cost_plot(diff_gap_theta*(iter_x-1)+iter_y,1:3) = [iter_x, iter_y, cost_res(diff_gap_theta*(iter_x-1)+iter_y)];
%     end
% end
% 
% if (idx_UAV == 1)
%     figure(3)
%     subplot(221), plot3(cost_plot(:,1),cost_plot(:,2),cost_plot(:,3),'b.');
% elseif (idx_UAV == 2)
%     figure(3)
%     subplot(222), plot3(cost_plot(:,1),cost_plot(:,2),cost_plot(:,3),'r.');
% elseif (idx_UAV == 3)
%     figure(3)
%     subplot(223), plot3(cost_plot(:,1),cost_plot(:,2),cost_plot(:,3),'g.');
% elseif (idx_UAV == 4)
%     figure(3)
%     subplot(224), plot3(cost_plot(:,1),cost_plot(:,2),cost_plot(:,3),'k.');
% end



