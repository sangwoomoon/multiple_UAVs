
clear all;
close all;
clc;

D2R = pi/180;

% UAV Spec.
NUM_UAV = 1;
INITIAL_VELOCITY = [5 5];
INITIAL_HEIGHT = [7 8];
FINAL_HEIGHT = [5 15];
MINIMUM_HEIGHT = [2 2];
x_rate = [-2 2];
y_rate = [-2 2];
z_rate = [-0.5 0.5];
roll_rate = [-50 50]*D2R;
pitch_rate = [-15 15]*D2R;
yaw_rate = [-60 60]*D2R;
linear_to_angular = [2 5 1];

% Obstacle Spec.
NUM_OBS = 7;
OBS_HEIGHT = [15 10 16 15 20 25 10];

% Scan Spec.
range_theta = [70 110]; % down - up, degrees
range_psi = [30 150]; % left - right, degrees
Search_range = 20;
d_angle = 0.2;  %radian

% Mission Requirement and Condition.
Cover_range = 3;    % m
initial_velocity = 5; % m/s
CHK_admissible = 3;   % m
dt = 0.2;


[OBS_Vert,Para_Plane] = Set_Environment(NUM_OBS,OBS_HEIGHT);

for iter_UAV = 1 : NUM_UAV
    [POS_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),VELO_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),POS_FINAL(iter_UAV,:)]...
        = Set_UAV(INITIAL_VELOCITY(iter_UAV),INITIAL_HEIGHT(iter_UAV),FINAL_HEIGHT(iter_UAV));
end


idx = 1;
POS_Scan = zeros(1,3);

clr = rand(1,3*NUM_UAV);

for iter_prcd = 1 : 300
    for iter_UAV = 1 : NUM_UAV
        [POS_Scan_tmp,idx_scan] = Detect_Obstacle(POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),...
            OBS_Vert,Para_Plane,NUM_OBS,OBS_HEIGHT,Search_range,range_theta,range_psi,d_angle);
        if (idx_scan > 1)
            POS_Scan(idx:idx+idx_scan-1,1:3) = POS_Scan_tmp;
            idx = idx + idx_scan;
        end
        clear POS_Scan_tmp;

        [POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6), VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)] = Motion_Planning(POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),...
            VELO_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),POS_Scan,Cover_range,POS_FINAL(iter_UAV,:),x_rate,y_rate,z_rate,linear_to_angular,dt,INITIAL_VELOCITY(iter_UAV),MINIMUM_HEIGHT(iter_UAV));
        
        figure(1)
        hold on; axis equal;
        plot3(POS_UAV(iter_prcd:iter_prcd+1,6*(iter_UAV-1)+1),-POS_UAV(iter_prcd:iter_prcd+1,6*(iter_UAV-1)+2),-POS_UAV(iter_prcd:iter_prcd+1,6*(iter_UAV-1)+3),'color',clr(3*(iter_UAV-1)+1:3*(iter_UAV-1)+3));
        
        if distance(POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+3),POS_FINAL(iter_UAV,:)) < CHK_admissible
            break;
        end
    end
end


figure(2)
grid on, axis equal; hold on;
if (idx > 1)
    plot3(POS_Scan(:,1),-POS_Scan(:,2),-POS_Scan(:,3),'r.');
end

for iter_UAV = 1 : NUM_UAV
    plot3(POS_UAV(:,6*(iter_UAV-1)+1),-POS_UAV(:,6*(iter_UAV-1)+2),-POS_UAV(:,6*(iter_UAV-1)+3),'color',rand(1,3));
end

% figure(3)
% x = 0 : 0.2 : 0.2*iter_prcd;
% for i = 1 : iter_prcd+1
%     V_mag(i) = norm(V(i,1:3));
% end
% plot(x,V_mag,x,V(:,1),x,V(:,2),x,V(:,3));
% 
% figure(4)
% plot(x,P(:,4)/D2R,x,P(:,5)/D2R,x,P(:,6)/D2R);
