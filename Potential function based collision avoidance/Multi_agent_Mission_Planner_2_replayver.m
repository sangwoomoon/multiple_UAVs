
% REVISION NOTE.

%  2010.02.25  Potential Field based planning algorithm was constructed.
%  2010.02.28  Master - Slave Distributed System was digged.
%  2010.03.11  A problem for avoidance was come out.
%              Avoidance function was a little bit changed :: summation
%  2010.03.12  Vertex Effect for avoidance was digged.

%  To Do
%  Line algorithm integration in Cluttered Environment.
%  Cul-de-Sac avoidance Algorithm
%  Dynamics and Control Integration.
%  MILP integration.

% ONLY FOR TASK ASSOIATED VERSION !!!

% clear all;
close all;
clc;

D2R = pi/180;

% UAV Spec.
NUM_UAV = 10;
INITIAL_VELOCITY = [5 5 5 5 5 5 5 5 5 5];
INITIAL_HEIGHT = [10 10 10 10 10 10 10 10 10 10];
FINAL_HEIGHT = [5 5 5 5 5 5 5 5 5 5];
MINIMUM_HEIGHT = 4*ones(1,NUM_UAV);
VELO_UAV_RATE = [1 1 1 1 1 1 1 1 1 1];
VELO_UAV_MAX = [10 10 10 10 10 10 10 10 10 10];

% Multi UAVs System.
GROUP_NUM = [1 1 2 2 2 3 3 3 4 4];     % Group number
MASTER_SLAVE = [1 0 1 0 0 1 0 0 1 0];  % MASTER = 1, SLAVE = 0;
D_desired = [0 5 5 5 5 5 5 5 5 5];     % Desired distance between Master and Slave.
POS_desired = [0 0 0; -1 1 0; -1 -1 0; 0 -1 0; -1 -1 0; 0 0 0; 0 1 0; -1 1 0; 0 -1 0; -1 -1 0 ];  % NED Coordiate

% Obstacle Spec.
NUM_OBS = 15;
OBS_MAX_HEIGHT = 30;
OBS_MIN_HEIGHT = 5;
OBS_HEIGHT = OBS_MIN_HEIGHT + (OBS_MAX_HEIGHT - OBS_MIN_HEIGHT)*rand(1,NUM_OBS);

% Scan Spec.
range_theta = [70 110]; % down - up, degrees
range_psi = [30 150]; % left - right, degrees
Search_range = 20;
d_angle = 0.2;  %radian

% Mission Requirement and Condition.
Cover_range = 3;    % m
initial_velocity = 5; % m/s
CHK_admissible = 5;   % m
dt = 0.2;

hold on;

for iter = 1 : NUM_TASK
    p_task(iter,2)=-p_task(iter,2);
    plot(p_task(iter,1),p_task(iter,2), 'g.','markersize',5);
    text(p_task(iter,1)-5,p_task(iter,2)-2,sprintf('%s %d %s','TASK',iter,'Point'));    
end

for iter_UAV = 1 : NUM_UAV
    if (iter_UAV ==1)
        [POS_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),VELO_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),POS_FINAL(iter_UAV,:)]...
            = Set_UAV_TaskMarged(INITIAL_VELOCITY(iter_UAV),INITIAL_HEIGHT(iter_UAV),FINAL_HEIGHT(iter_UAV),iter_UAV,[p_i(1,:);p_f(1,:);HLI_pos(2,1:2)],0);
    elseif (iter_UAV ==3)
        [POS_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),VELO_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),POS_FINAL(iter_UAV,:)]...
            = Set_UAV_TaskMarged(INITIAL_VELOCITY(iter_UAV),INITIAL_HEIGHT(iter_UAV),FINAL_HEIGHT(iter_UAV),iter_UAV,[p_i(2,:);p_f(2,:);HLI_pos(2,4:5)],0);        
    elseif (iter_UAV ==6)
        [POS_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),VELO_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),POS_FINAL(iter_UAV,:)]...
            = Set_UAV_TaskMarged(INITIAL_VELOCITY(iter_UAV),INITIAL_HEIGHT(iter_UAV),FINAL_HEIGHT(iter_UAV),iter_UAV,[p_i(3,:);p_f(3,:);HLI_pos(2,7:8)],0);        
    elseif (iter_UAV ==9)
        [POS_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),VELO_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),POS_FINAL(iter_UAV,:)]...
            = Set_UAV_TaskMarged(INITIAL_VELOCITY(iter_UAV),INITIAL_HEIGHT(iter_UAV),FINAL_HEIGHT(iter_UAV),iter_UAV,[p_i(4,:);p_f(4,:);HLI_pos(2,10:11)],0);
    else
        [POS_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),VELO_UAV(1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),POS_FINAL(iter_UAV,:)]...
            = Set_UAV_TaskMarged(INITIAL_VELOCITY(iter_UAV),INITIAL_HEIGHT(iter_UAV),FINAL_HEIGHT(iter_UAV),iter_UAV,[p_i(4,:);p_f(4,:);HLI_pos(2,10:11)],1);        
    end
end


[OBS_Vert,Para_Plane] = Set_Environment_TaskMarged(NUM_OBS,OBS_HEIGHT,vrt);


idx = 1;
POS_Scan = zeros(1,3);

clr = rand(1,3*NUM_UAV);

iter_prcd = 1;
iter_UAV = 1;
FLAG_OPERATE = 1;   % operation will be proceed if this value is 1.
SWITCH_OPERATE = zeros(1,NUM_UAV);  % if a UAV reaches to the goal point, the number will be switched to 1.
TASK_Perform = 50*ones(1,NUM_UAV);  % task doing for 10 time steps.
iter_task = ones(1,NUM_UAV);
while (FLAG_OPERATE)
    
    % check distance between two agents.
    if (NUM_UAV > 1)
        idx_dist = 1;
        for iter_1 = 1 : NUM_UAV-1
            for iter_2 = iter_1+1 : NUM_UAV
                POS_UAV_DIST(iter_prcd,idx_dist) = distance(POS_UAV(iter_prcd,6*(iter_1-1)+1:6*(iter_1-1)+3),POS_UAV(iter_prcd,6*(iter_2-1)+1:6*(iter_2-1)+3));
                idx_dist = idx_dist + 1;
            end
        end
    end
    
    % detect and planning
    for iter_UAV = 1 : NUM_UAV
        if (iter_UAV < 3)
            POS_TaskNow = POS_TASK(iter_task(iter_UAV),1:2);
        elseif (iter_UAV < 6)
            POS_TaskNow = POS_TASK(iter_task(iter_UAV),5:7);
        elseif (iter_UAV < 9)
            POS_TaskNow = POS_TASK(iter_task(iter_UAV),9:11);
        else
            POS_TaskNow = POS_TASK(iter_task(iter_UAV),13:15);
        end
        
        if (distance(POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+2),POS_TaskNow) > CHK_admissible)
            [POS_Scan_tmp,idx_scan] = ...
                Detect_Obstacle(POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6),OBS_Vert,Para_Plane,NUM_OBS,OBS_HEIGHT,Search_range,range_theta,range_psi,d_angle);
            if (idx_scan > 1)
                POS_Scan(idx:idx+idx_scan-1,1:5) = [POS_Scan_tmp iter_UAV*ones(idx_scan,1) iter_prcd*ones(idx_scan,1)];
                idx = idx + idx_scan;
            end
            
            if (iter_UAV < 3)
                [POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6), VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)] = ...
                    Motion_Planning_2(POS_UAV(iter_prcd,:),VELO_UAV(iter_prcd,:),iter_UAV,NUM_UAV,POS_Scan_tmp,Cover_range,POS_TASK(iter_task(iter_UAV),1:3),dt,...
                    INITIAL_VELOCITY(iter_UAV),D_desired(iter_UAV),VELO_UAV_MAX(iter_UAV),VELO_UAV_RATE(iter_UAV),MINIMUM_HEIGHT(iter_UAV),GROUP_NUM,MASTER_SLAVE,POS_desired(iter_UAV,:));
            elseif (iter_UAV < 6)
                [POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6), VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)] = ...
                    Motion_Planning_2(POS_UAV(iter_prcd,:),VELO_UAV(iter_prcd,:),iter_UAV,NUM_UAV,POS_Scan_tmp,Cover_range,POS_TASK(iter_task(iter_UAV),5:7),dt,...
                    INITIAL_VELOCITY(iter_UAV),D_desired(iter_UAV),VELO_UAV_MAX(iter_UAV),VELO_UAV_RATE(iter_UAV),MINIMUM_HEIGHT(iter_UAV),GROUP_NUM,MASTER_SLAVE,POS_desired(iter_UAV-2,:));                
            elseif (iter_UAV < 9)
                [POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6), VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)] = ...
                    Motion_Planning_2(POS_UAV(iter_prcd,:),VELO_UAV(iter_prcd,:),iter_UAV,NUM_UAV,POS_Scan_tmp,Cover_range,POS_TASK(iter_task(iter_UAV),9:11),dt,...
                    INITIAL_VELOCITY(iter_UAV),D_desired(iter_UAV),VELO_UAV_MAX(iter_UAV),VELO_UAV_RATE(iter_UAV),MINIMUM_HEIGHT(iter_UAV),GROUP_NUM,MASTER_SLAVE,POS_desired(iter_UAV-5,:));                
            else
                [POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6), VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)] = ...
                    Motion_Planning_2(POS_UAV(iter_prcd,:),VELO_UAV(iter_prcd,:),iter_UAV,NUM_UAV,POS_Scan_tmp,Cover_range,POS_TASK(iter_task(iter_UAV),13:15),dt,...
                    INITIAL_VELOCITY(iter_UAV),D_desired(iter_UAV),VELO_UAV_MAX(iter_UAV),VELO_UAV_RATE(iter_UAV),MINIMUM_HEIGHT(iter_UAV),GROUP_NUM,MASTER_SLAVE,POS_desired(iter_UAV-8,:));                
            end

            clear POS_Scan_tmp;
        else
            if (iter_UAV <3)
                TASK_flag = POS_TASK(iter_task(iter_UAV),4);
                if (MASTER_SLAVE(iter_UAV) == 0)
                   TASK_Perform(iter_UAV)= TASK_Perform(1);
                   iter_task(iter_UAV)=iter_task(1);
                end
            elseif (iter_UAV <6)
                TASK_flag = POS_TASK(iter_task(iter_UAV),8);
                if (MASTER_SLAVE(iter_UAV) == 0)
                   TASK_Perform(iter_UAV)= TASK_Perform(3);
                   iter_task(iter_UAV)=iter_task(3);
                end                
            elseif (iter_UAV <9)
                TASK_flag = POS_TASK(iter_task(iter_UAV),12);
                if (MASTER_SLAVE(iter_UAV) == 0)
                   TASK_Perform(iter_UAV)= TASK_Perform(6);
                   iter_task(iter_UAV)=iter_task(6);
                end                
            else
                TASK_flag = POS_TASK(iter_task(iter_UAV),16);
                if (MASTER_SLAVE(iter_UAV) == 0)
                   TASK_Perform(iter_UAV)= TASK_Perform(9);
                   iter_task(iter_UAV)=iter_task(9);
                end
            end
            
            if (TASK_flag==1)
                if (TASK_Perform(iter_UAV)>0)
                    POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6);
                    VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = zeros(1,6);
                    if (MASTER_SLAVE(iter_UAV) == 1)
                        TASK_Perform(iter_UAV)=TASK_Perform(iter_UAV)-1;
                    end
                else
                    POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)+VELO_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)*dt;
                    VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = VELO_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6);
                    if (MASTER_SLAVE(iter_UAV) == 1)
                        iter_task(iter_UAV) = iter_task(iter_UAV)+1;
                        TASK_Perform(iter_UAV) = 50;
                    end
                end
            else
                if (iter_task(iter_UAV) == 8) %MAXIMUM
                    POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6);
                    VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = zeros(1,6);
                else
                    POS_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = POS_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)+VELO_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6)*dt;
                    VELO_UAV(iter_prcd+1,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6) = VELO_UAV(iter_prcd,6*(iter_UAV-1)+1:6*(iter_UAV-1)+6);
                    if (MASTER_SLAVE(iter_UAV) == 1)
                        iter_task(iter_UAV) = iter_task(iter_UAV)+1;
                        TASK_Perform(iter_UAV) = 50;
                    end
                end
            end
        end
        
        % check the mission requirement.
        CHK_SUM = 0;
        for iter_chk = 1 : NUM_UAV
            CHK_SUM = CHK_SUM + SWITCH_OPERATE(iter_chk);
        end
        if (CHK_SUM == 1)    %%%% 수정 필요
            FLAG_OPERATE = 0;
        end
        
        if (iter_UAV <3)
            TASK_flag = POS_TASK(iter_task(iter_UAV),4);
            if (MASTER_SLAVE(iter_UAV) == 0)
                TASK_Perform(iter_UAV)= TASK_Perform(1);
                iter_task(iter_UAV)=iter_task(1);
            end
        elseif (iter_UAV <6)
            TASK_flag = POS_TASK(iter_task(iter_UAV),8);
            if (MASTER_SLAVE(iter_UAV) == 0)
                TASK_Perform(iter_UAV)= TASK_Perform(3);
                iter_task(iter_UAV)=iter_task(3);
            end
        elseif (iter_UAV <9)
            TASK_flag = POS_TASK(iter_task(iter_UAV),12);
            if (MASTER_SLAVE(iter_UAV) == 0)
                TASK_Perform(iter_UAV)= TASK_Perform(6);
                iter_task(iter_UAV)=iter_task(6);
            end
        else
            TASK_flag = POS_TASK(iter_task(iter_UAV),16);
            if (MASTER_SLAVE(iter_UAV) == 0)
                TASK_Perform(iter_UAV)= TASK_Perform(9);
                iter_task(iter_UAV)=iter_task(9);
            end
        end

        figure(1)
        hold on; axis equal;
        plot3(POS_UAV(iter_prcd:iter_prcd+1,6*(iter_UAV-1)+1),-POS_UAV(iter_prcd:iter_prcd+1,6*(iter_UAV-1)+2),-POS_UAV(iter_prcd:iter_prcd+1,6*(iter_UAV-1)+3),'color',clr(3*(iter_UAV-1)+1:3*(iter_UAV-1)+3));
    end
    iter_prcd = iter_prcd + 1;
    
%     if iter_prcd == 500
%         break;
%     end
end

% figure(2)
% grid on, axis equal; hold on;
% if (idx > 1)
%     plot3(POS_Scan(:,1),-POS_Scan(:,2),-POS_Scan(:,3),'r.');
% end

for iter_UAV = 1 : NUM_UAV
    plot3(POS_UAV(:,6*(iter_UAV-1)+1),-POS_UAV(:,6*(iter_UAV-1)+2),-POS_UAV(:,6*(iter_UAV-1)+3),'color',rand(1,3));
end

figure(4)
t = 0:dt:(iter_prcd-1)*dt;
plot(t,POS_UAV_DIST);

