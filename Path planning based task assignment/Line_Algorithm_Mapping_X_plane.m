
%%% Vertex Database Type %%%
%%% [ x y idx_bldg idx_edge invite determinent_bldg invite determinent_edge] %%%

%%% Waypoint Database Type %%%
%%% [ x y idx_bldg idx_edge] %%%


%%%  TO DO %%%
%%% Dynamic path 를 구할 때 cover range 안에 들어가는 거 처리
%%% CUL-DE-SAC 처리



% 
clear all;
close all;
clc;

load Obstacles_LLH.mat

% Obstacles_LLH(:,2:3) = Obstacles_LLH(:,2:3) ;

D2R = pi/180;


Num_pt = size(Obstacles_LLH,1);

for i= 1:Num_pt 

    lat = Obstacles_LLH(i,2);
    long = Obstacles_LLH(i,3);
    h = 0 ; 
    [XYZ(i,1), XYZ(i,2), XYZ(i,3)] = llh2xyz(lat*D2R,long*D2R,h); 

end
XYZ_ref = XYZ(1,:);
% figure; plot3( XYZ(:,1), XYZ(:,2), XYZ(:,3),'*'); hold on;
% figure; plot(( XYZ(:,1)-XYZ_ref(1)), (XYZ(:,2)-XYZ_ref(2)),'*'); hold on;



for i = 1:Num_pt
    
    [NED(i,1), NED(i,2), NED(i,3)]=xyz2ned(XYZ_ref(1),XYZ_ref(2),XYZ_ref(3),XYZ(i,1), XYZ(i,2), XYZ(i,3));
        
end

% figure; plot(NED(:,2),NED(:,1),'+');

obstacle_oem = [NED(:,2) NED(:,1)] *0.02 ;
obstacle_oem(:,1) = obstacle_oem(:,1)  ; 
obstacle_oem(:,2) = obstacle_oem(:,2)  ; 

vrt(1:3,1:2) = obstacle_oem(2:4,:);
vrt(4,1:2) = obstacle_oem(8,:);
vrt(5,1:2) = obstacle_oem(2,:);
vrt(1:5,3) = 1 ; 

vrt(6:9,1:2) = obstacle_oem(4:7,:);
vrt(10,1:2) = obstacle_oem(4,:);
vrt(6:10,3) = 2 ; 

vrt(11:14,1:2) = obstacle_oem(9:12,:);
vrt(15,1:2) = obstacle_oem(9,:);
vrt(11:15,3) = 3 ; 

vrt(16:19,1:2) = obstacle_oem(13:16,:);
vrt(20,1:2) = obstacle_oem(13,:);
vrt(16:20,3) = 4 ; 

vrt(21:24,1:2) = obstacle_oem(17:20,:);
vrt(25,1:2) = obstacle_oem(17,:);
vrt(21:25,3) = 5 ; 

p_i(1:2,1:2) = obstacle_oem(21:22,:);
p_f(1:2,1:2) = obstacle_oem(23:24,:);

p_i(1,:) = [-200 25]; 
% p_i(3,:) = [-100 25];
% p_f(3,:) = [-100 300];


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment and Condition Spec. %%%%%%%%%%%%%%%%%%%%%%%
% known environment.
OBS_NUM = 5;
OBS_VRT = 4;

% unknown environment.
UNOBS_NUM = 0;
UNOBS_VRT = 4;

COV_RANGE = 3; % margin to set the configuration space.
chk_range = 2; % m, required range to reach to the goal.
dt = 0.4;    % Time Interval
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Helicopter Spec. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NUM_UAV = 3;
SCAN_RANGE = 20*ones(1,NUM_UAV); %[20 20 20 20 20 20 20];
SCAN_THETA = 30*pi/180*ones(1,NUM_UAV); %[30*pi/180, 30*pi/180 30*pi/180 30*pi/180 30*pi/180 30*pi/180];

HLI_vel = ones(1,NUM_UAV)*1; %[1 1 1 1 1 1]; % m/s
HLI_max_dyaw = 20*pi/180*ones(1,NUM_UAV); %[0.4 0.4 0.4 0.4 0.4 0.4]; %rad/s
cnt(1:NUM_UAV) = chk_range / dt+3; % count to prceed to the next step.
cnt_terminate = zeros(1,NUM_UAV);  % binary value for mission requirement
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Task Spec.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
NUM_TASK = 6 ;
TASK_TIME = 1; % second
TASK_STATUS = zeros(2,NUM_TASK); % 0:Not yet, 1: Scheduled, 2:Doing 3:Done // 2nd tag::UAV#
SetTaskAllocation = [1,0];  % event trigger (1 : TA OPERATION), 2nd col:triggered UAV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Movie File Spec. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
det_mk_movie = 0; % 0 : do not make, 1 : make
if (det_mk_movie)
    frame_per_sec = 10; % fps. 15 is default in Matlab
    file_name = 'movie_file.avi';
    codeck = 'IVUY'; % compressing codeck
    quality = 100;  % quality of file. 100 is maximum.
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL CONDITION SETTEING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% axis([-400 200 -100 400])
% set(gcf,'color',[1 1 1]);
% grid on
% hold on
% axis equal tight

fig = figure(1);
set(fig,'DoubleBuffer','on');


j = 1 ; 
for i = 1: size(vrt,1)
   vrt(i,4) = j ; 
   if j == 5 
       vrt(i,4) = 1 ; 
       j = j - 5 ; 
   else 
       line(vrt(i:i+1,1),vrt(i:i+1,2)); hold on; 
   end
   j = j + 1 ; 
    
end

if (OBS_NUM == 0)
    title('Realtime Path Planning in Totally Unknown Environment'); %, Sangwoo Moon, 2010');
elseif (UNOBS_NUM == 0)
    title('Path Planning in Totally Known Environment');
else
    title('Realtime Path Planning in Cluttered Environment'); %, Sangwoo Moon, 2010');
end
xlabel('X direction (m)');
ylabel('Y direction (m)');
zlabel('Z direction (m)');
% uicontrol('style','text','position',[60 60 80 20],'string','t=','backgroundcolor',[0.8 0.8 0.8]);

index_clr = rand(1,NUM_UAV*3); % waypoint color

p_i = [ -100 50 ; 25 50 ; 100 50 ] ; 
p_f = [-50 250 ; 0 250 ; 50 250 ] ;
p_task = [-256.8548 241.5205 ;
    -211.6935 323.3918;
    -143.9516 288.3041;
    -6.8548 200.5848;
    -53.6290 250.8772;
    -34.2742 318.7135 ]; 

for iter = 1 : NUM_UAV
    %p_i(iter,1:2) = [-60 40]+[20*rand(1) 10*rand(1)];
%     [p_i(iter,1), p_i(iter,2)] = ginput(1);% Initial Point
    plot(p_i(iter,1),p_i(iter,2), 'ro');
    text(p_i(iter,1)-5,p_i(iter,2)-2,sprintf('%s %d %s','UAV',iter,'Start'));

    %p_f(iter,1:2) = [60 -40]+[-20*rand(1) -10*rand(1)];
%     [p_f(iter,1), p_f(iter,2)] = ginput(1);% Final Point
    plot(p_f(iter,1),p_f(iter,2), 'bo');
    text(p_f(iter,1),p_f(iter,2)-2,sprintf('%s %d %s','UAV',iter,'Goal'));
end

% [-60 60 40 50]
% [-60 60 -40 -50]


% Known Obstacles setting and calculate Configuration Space for known obstacles
if (OBS_NUM ~= 0)
%     vrt = set_obstacle(OBS_NUM,OBS_VRT,0,1);
    vrt_config = set_config_space(vrt,OBS_NUM,OBS_VRT,COV_RANGE);
end
% Unknown Obstacles setting and calculate Configuration Space for unknown obstacles

if (UNOBS_NUM ~= 0)
    vrt_unknown = set_obstacle(UNOBS_NUM,UNOBS_VRT,OBS_NUM,0);
    vrt_config_unknown = set_config_space(vrt_unknown,UNOBS_NUM,UNOBS_VRT,COV_RANGE);
    % determine whether this unknown obstacle was drawn by CW or CCW. this data
    % will be used when detected line is deformed to the configuration space.
    cw_ccw = zeros(1,UNOBS_NUM);
    for iter_obs = 1 : UNOBS_NUM
        cw_ccw(iter_obs) = determine_cw_ccw(vrt_unknown((iter_obs-1)*(UNOBS_VRT+1)+1:iter_obs*(UNOBS_VRT+1),1:2));
    end
end

% Task Setting
for iter = 1 : NUM_TASK
%     [p_task(iter,1),p_task(iter,2)] = ginput(1); % task point
    plot(p_task(iter,1),p_task(iter,2), 'g.','markersize',5);
    text(p_task(iter,1)-5,p_task(iter,2)-2,sprintf('%s %d %s','TASK',iter,'Point'));    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ANIMATION SETTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helicopter scale.
scale = 0.3;

%Draw RUAV :: head + rotor + boom + tail
heli_head=[-2 2; -2 -2; 5 0]*scale;
heli_boom = [-2 0;-10 0]*scale;
heli_tail = [-10,0;-11,2;-9,0;-11,-2;-10,0]*scale;

part_num = 40;
for idx=1:part_num
   x=6*cos(2*pi/part_num*idx)*scale;
   y=6*sin(2*pi/part_num*idx)*scale;
   heli_rotor(idx,1)=x;
   heli_rotor(idx,2)=y;
end
heli_rotor(part_num+1,:)=heli_rotor(1,:);

for iter_range = 1 : NUM_UAV
    set_pt = [0,0,0];
    set_bnd((iter_range-1)*4+1:(iter_range-1)*4+3,1:2) = [set_pt(1:2);
        set_pt(1,1)+SCAN_RANGE(iter_range)*cos(set_pt(1,3)-SCAN_THETA(iter_range)),set_pt(1,2)+SCAN_RANGE(iter_range)*sin(set_pt(1,3)-SCAN_THETA(iter_range));
        set_pt(1,1)+SCAN_RANGE(iter_range)*cos(set_pt(1,3)+SCAN_THETA(iter_range)),set_pt(1,2)+SCAN_RANGE(iter_range)*sin(set_pt(1,3)+SCAN_THETA(iter_range))];
    set_bnd(iter_range*4,:) = set_bnd((iter_range-1)*4+1,:);
end

% RUAV Assign
for idx = 1 : NUM_UAV
    clr = rand(1,3);
    head(idx) = patch(heli_head(:,1),heli_head(:,2),clr);
    rotor(idx) = line(heli_rotor(:,1),heli_rotor(:,2),'color','black');
    boom(idx) = line(heli_boom(:,1),heli_boom(:,2),'color','black','linewidth',2);
    tail(idx) = patch(heli_tail(:,1),heli_tail(:,2),rand(1,3));
    if (UNOBS_NUM ~= 0)
        handle_srh(idx) = plot(set_bnd((idx-1)*4+1:idx*4,1),set_bnd((idx-1)*4+1:idx*4,2),'color','red');
    end
    handle_waypoint(idx) = plot(set_pt(1,1),set_pt(1,2),'Marker','*');
    % handle_second_wpt(idx) = plot(set_pt(1,1),set_pt(1,2),'Marker','o');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Makinng Movie Clip %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (det_mk_movie)
   aviobj = avifile(file_name,'fps',frame_per_sec,'Quality',quality); %'compression','IVUY',
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN PROCEDURE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
% Find Path. in Cluttered Enviornment, the path should be solved by
% dynamically assigned. therefore, we only use the first segment of the
% whole path.
% Then, find the dynamic path which considers the kinamatics of UAV and
% search the unknown environment. if edges are detected, this data are
% updated to the vertex database which contains about the location of
% obstacles.
% if somethings are detected or an UAV reaches to the other waypoint on a
% path segment, regenerate the path. (dynamic path generation)
%

meet_pts = zeros(1,6);  % searching points
scan_point = zeros(1,6);
iter_prcd = 1;
iter_wpt(1:NUM_UAV) = 2;
DistPathForTask = inf*ones(NUM_UAV,NUM_TASK);
MinDistTask = zeros(1,NUM_UAV);
idxMinTask = zeros(1,NUM_UAV);
while (1)
    
    tic;
    
    if (iter_prcd == 1)
        for iter_UAV = 1 : NUM_UAV
            HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3) = [p_i(iter_UAV,:) 0];
            wpt(1,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(1,4);
            second_choice(1,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(1,4);
        end
    end
        
    for iter_UAV = 1 : NUM_UAV
        if (SetTaskAllocation(1))
            for iter_TASK = 1 : NUM_TASK
                % first and second waypoints are olny used.
                if (TASK_STATUS(1,iter_TASK) ~= 3)
                    [wpt_temp, second_choice_temp, DistPathForTask(iter_UAV,iter_TASK)] = ...
                        operate_algorithm(OBS_NUM,OBS_VRT,wpt(1,(iter_UAV-1)*4+1:iter_UAV*4),...
                        vrt_config,vrt_config,...
                        HLI_pos(iter_prcd,(iter_UAV-1)*3+1:(iter_UAV-1)*3+2),p_task(iter_TASK,:),...
                        1);
                    if (length(wpt_temp(:,1)) == 2)
                        DistPathForTask(iter_UAV,iter_TASK) = norm(wpt_temp(2,1:2) - wpt_temp(1,1:2));
                    end
                    DistPathForTask(iter_UAV,iter_TASK) = DistPathForTask(iter_UAV,iter_TASK); % + norm(wpt_temp(length(wpt_temp(:,1)),1:2)-p_f(iter_UAV,1:2));
                else
                    DistPathForTask(iter_UAV,iter_TASK) = inf;
                end
                % find the best choice for the task assignment (fully distributed)
                idxMinTask(iter_UAV) = 1;
                MinDistTask(iter_UAV) = DistPathForTask(iter_UAV,1);
                for iter_TASK = 2 : NUM_TASK
                    if (DistPathForTask(iter_UAV,iter_TASK) < MinDistTask(iter_UAV))
                        MinDistTask(iter_UAV) = DistPathForTask(iter_UAV,iter_TASK);
                        idxMinTask(iter_UAV) = iter_TASK;
                    end
                end
            end
        end

        % unknown obstacle scanning and data merging if there are unknown
        % obstacles.
        if (UNOBS_NUM ~= 0)
            [meet_pts_tmp,flag_meet_pts,srh_bound] = scan_obstacle(HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3),vrt_unknown(:,1:4),OBS_NUM,UNOBS_NUM,UNOBS_VRT,SCAN_RANGE(iter_UAV),SCAN_THETA(iter_UAV));
            if (flag_meet_pts == 1)
                [search_pts,meet_pts,expand_pts] = data_update(meet_pts,meet_pts_tmp,cw_ccw,OBS_NUM,COV_RANGE,UNOBS_VRT,vrt_config_unknown);
            end
        else
            flag_meet_pts = 0;
        end

        % waypoint proceeding and regenerate path.
%         if (flag_meet_pts == 1)
%             [wpt_temp, second_choice_temp, temp_1] = ...
%                 operate_algorithm(OBS_NUM,OBS_VRT,wpt(:,(iter_UAV-1)*4+1:(iter_UAV-1)*4+2),...
%                 [vrt_config;expand_pts(:,1:4),zeros(length(expand_pts(:,1)),2)],[vrt_config;meet_pts(:,1:4),zeros(length(meet_pts(:,1)),2)],...
%                 HLI_pos(iter_prcd,(iter_UAV-1)*3+1:(iter_UAV-1)*3+2),p_task(iter_task,:),iter_wpt(iter_UAV)-1); % p_task should be modified
% 
%             wpt(1:length(wpt_temp(:,1)),(iter_UAV-1)*4+1:(iter_UAV*4)) = wpt_temp;
%             second_choice(1,(iter_UAV-1)*4+1:iter_UAV*4) = second_choice_temp;
%             clear wpt_temp second_choice_temp;
%         end
    end

    % Task Re-assignment
    if (SetTaskAllocation(1))        
        for iter_UAVx = 1 : NUM_UAV-1
            for iter_UAVy = iter_UAVx+1 : NUM_UAV
                if ((MinDistTask(iter_UAVx) ~= inf) || (MinDistTask(iter_UAVy) ~= inf))
                    if (idxMinTask(iter_UAVx) == idxMinTask(iter_UAVy))
                        if (MinDistTask(iter_UAVx) < MinDistTask(iter_UAVy))
                            TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.
                            MinDistTask(iter_UAVy) = inf;
                            for iter_TASK = 1 : NUM_TASK
                                if (DistPathForTask(iter_UAVy,iter_TASK) < MinDistTask(iter_UAVy)) && (TASK_STATUS(1,iter_TASK) == 0)
                                    MinDistTask(iter_UAVy) = DistPathForTask(iter_UAVy,iter_TASK);
                                    idxMinTask(iter_UAVy) = iter_TASK;
                                end
                            end
                            if MinDistTask(iter_UAVy) == inf
                                [wpt_temp, second_choice_temp, temp] = ...
                                    operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
                                    vrt_config,vrt_config,...
                                    HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_f(iter_UAVy,:),...
                                    iter_wpt(iter_UAVy)-1);
                                wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
                                second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
                            else
                                [wpt_temp, second_choice_temp, temp] = ...
                                    operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
                                    vrt_config,vrt_config,...
                                    HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_task(idxMinTask(iter_UAVy),:),...
                                    iter_wpt(iter_UAVy)-1);
                                wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
                                second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
                                TASK_STATUS(:,idxMinTask(iter_UAVy)) = [1;iter_UAVy]; % TASK status update.
                            end
                        else
                            TASK_STATUS(:,idxMinTask(iter_UAVy)) = [1;iter_UAVy]; % TASK status update.
                            MinDistTask(iter_UAVx) = inf;
                            for iter_TASK = 1 : NUM_TASK
                                if (DistPathForTask(iter_UAVx,iter_TASK) < MinDistTask(iter_UAVx)) && (TASK_STATUS(1,iter_TASK) == 0)
                                    MinDistTask(iter_UAVx) = DistPathForTask(iter_UAVx,iter_TASK);
                                    idxMinTask(iter_UAVx) = iter_TASK;
                                end
                            end
                            if MinDistTask(iter_UAVx) == inf
                                [wpt_temp, second_choice_temp, temp] = ...
                                    operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVx)-1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
                                    vrt_config,vrt_config,...
                                    HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_f(iter_UAVx,:),...
                                    iter_wpt(iter_UAVx)-1);
                                wpt(1:length(wpt_temp(:,1)),(iter_UAVx-1)*4+1:(iter_UAVx*4)) = wpt_temp;
                                second_choice(1,(iter_UAVx-1)*4+1:iter_UAVx*4) = second_choice_temp;
                            else
                                [wpt_temp, second_choice_temp, temp] = ...
                                    operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVx)-1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
                                    vrt_config,vrt_config,...
                                    HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_task(idxMinTask(iter_UAVx),:),...
                                    iter_wpt(iter_UAVx)-1);
                                wpt(1:length(wpt_temp(:,1)),(iter_UAVx-1)*4+1:(iter_UAVx*4)) = wpt_temp;
                                second_choice(1,(iter_UAVx-1)*4+1:iter_UAVx*4) = second_choice_temp;
                                TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.
                            end
                        end

                        clear wpt_temp second_choice_temp;
                    else
                        [wpt_temp, second_choice_temp, temp] = ...
                            operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVx)-1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
                            vrt_config,vrt_config,...
                            HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_task(idxMinTask(iter_UAVx),:),...
                            iter_wpt(iter_UAVx)-1);
                        wpt(1:length(wpt_temp(:,1)),(iter_UAVx-1)*4+1:(iter_UAVx*4)) = wpt_temp;
                        second_choice(1,(iter_UAVx-1)*4+1:iter_UAVx*4) = second_choice_temp;
                        clear wpt_temp second_choice_temp;
                        TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.

                        [wpt_temp, second_choice_temp, temp] = ...
                            operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
                            vrt_config,vrt_config,...
                            HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_task(idxMinTask(iter_UAVy),:),...
                            iter_wpt(iter_UAVy)-1);
                        wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
                        second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
                        clear wpt_temp second_choice_temp;
                        TASK_STATUS(:,idxMinTask(iter_UAVy)) = [1;iter_UAVy]; % TASK status update.
                    end
                else
                    [wpt_temp, second_choice_temp, temp] = ...
                        operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVx)-1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
                        vrt_config,vrt_config,...
                        HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_f(iter_UAVx,:),...
                        iter_wpt(iter_UAVx)-1);
                    wpt(1:length(wpt_temp(:,1)),(iter_UAVx-1)*4+1:(iter_UAVx*4)) = wpt_temp;
                    second_choice(1,(iter_UAVx-1)*4+1:iter_UAVx*4) = second_choice_temp;
                    clear wpt_temp second_choice_temp;

                    [wpt_temp, second_choice_temp, temp] = ...
                        operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
                        vrt_config,vrt_config,...
                        HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_f(iter_UAVy,:),...
                        iter_wpt(iter_UAVy)-1);
                    wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
                    second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
                    clear wpt_temp second_choice_temp;
                end
            end
        end
        chk_FINAL = 0;
        for iter_FINAL = 1 : NUM_UAV
            if (MinDistTask(iter_FINAL) ~= inf)
                chk_FINAL = chk_FINAL + 1;
            end
        end
        if chk_FINAL == 0 % final state (the last UAV should go to the goal point)
            [wpt_temp, second_choice_temp, temp] = ...
                operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(SetTaskAllocation(2))-1,(SetTaskAllocation(2)-1)*4+1:SetTaskAllocation(2)*4),...
                vrt_config,vrt_config,...
                HLI_pos(iter_prcd,(SetTaskAllocation(2)-1)*3+1:(SetTaskAllocation(2)-1)*3+2),p_f(idxMinTask(SetTaskAllocation(2)),:),...
                iter_wpt(iter_UAV)-1);
            wpt(1:length(wpt_temp(:,1)),(SetTaskAllocation(2)-1)*4+1:(SetTaskAllocation(2)*4)) = wpt_temp;
            second_choice(1,(SetTaskAllocation(2)-1)*4+1:SetTaskAllocation(2)*4) = second_choice_temp;
            clear wpt_temp second_choice_temp;
       end
    end

    % calculate the UAV position
    for iter_UAV = 1 : NUM_UAV
        if (cnt_terminate(iter_UAV) == 0)
            [HLI_pos(iter_prcd+1,(iter_UAV-1)*3+1:iter_UAV*3),flag_change] = ...
                calculate_postion_xplane(HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3),wpt(iter_wpt(iter_UAV),(iter_UAV-1)*4+1:(iter_UAV-1)*4+2),HLI_vel(iter_UAV),HLI_max_dyaw(iter_UAV),dt,chk_range);
            if (flag_change == 1)
                cnt(iter_UAV) = cnt(iter_UAV) - 1;
                if cnt(iter_UAV) == 0
                    iter_wpt(iter_UAV) = iter_wpt(iter_UAV) + 1;
                    cnt(iter_UAV) = chk_range/dt+3;
                end
            end
        else
            HLI_pos(iter_prcd+1,(iter_UAV-1)*3+1:iter_UAV*3) = HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3);
        end
    end

    SetTaskAllocation(1) = 0;  % event trigger!

    for idx_UAV = 1 : NUM_UAV
        POS_UAV_X(idx_UAV) = HLI_pos(iter_prcd,3*(idx_UAV-1)+1);
        POS_UAV_Y(idx_UAV) = HLI_pos(iter_prcd,3*(idx_UAV-1)+2);
    end
    
    time_cal(iter_prcd) = toc;
    
    tic;
    
    for idx_UAV = 1 : NUM_UAV
        % Angle Assign 
        euler = [cos(HLI_pos(iter_prcd,3*(idx_UAV-1)+3)),-sin(HLI_pos(iter_prcd,3*(idx_UAV-1)+3));sin(HLI_pos(iter_prcd,3*(idx_UAV-1)+3)),cos(HLI_pos(iter_prcd,3*(idx_UAV-1)+3))];
        heli_head_trans=[heli_head(:,1) heli_head(:,2)]*euler';
        heli_boom_trans = [heli_boom(:,1) heli_boom(:,2)]*euler';
        heli_tail_trans = [heli_tail(:,1) heli_tail(:,2)]*euler';
        heli_rotor_trans = [heli_rotor(:,1) heli_rotor(:,2)]*euler';
        heli_srh_trans = [set_bnd(:,1),set_bnd(:,2)]*euler';
        
        set(head(idx_UAV),'Xdata',heli_head_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_head_trans(:,2)+POS_UAV_Y(idx_UAV));
        set(rotor(idx_UAV),'Xdata',heli_rotor_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_rotor_trans(:,2)+POS_UAV_Y(idx_UAV));
        set(boom(idx_UAV),'Xdata',heli_boom_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_boom_trans(:,2)+POS_UAV_Y(idx_UAV));
        set(tail(idx_UAV),'Xdata',heli_tail_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_tail_trans(:,2)+POS_UAV_Y(idx_UAV));
%         set(handle_waypoint(idx_UAV),'XData',wpt(iter_wpt(idx_UAV),(idx_UAV-1)*4+1),'YData',wpt(iter_wpt(idx_UAV),(idx_UAV-1)*4+2),'Color',index_clr((idx_UAV-1)*3+1:idx_UAV*3));
        % set(handle_second_wpt(idx_UAV),'XData',second_choice(1,(idx_UAV-1)*4+1),'YData',second_choice(1,(idx_UAV-1)*4+2),'Color',index_clr((idx_UAV-1)*3+1:idx_UAV*3));
        if (UNOBS_NUM ~= 0)
            set(handle_srh(idx_UAV),'Xdata',heli_srh_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_srh_trans(:,2)+POS_UAV_Y(idx_UAV));
        end
        % plot(HLI_pos([iter_prcd,iter_prcd+1],(idx_UAV-1)*3+1),HLI_pos([iter_prcd,iter_prcd+1],(idx_UAV-1)*3+2),'k-');
    end
    drawnow; 
    
    for iter_scan = 1 : 2 : length(meet_pts(:,1))-1
        plot(search_pts(iter_scan:iter_scan+1,1),search_pts(iter_scan:iter_scan+1,2),'r-','linewidth',2);
        % plot(meet_pts(iter_scan:iter_scan+1,1),meet_pts(iter_scan:iter_scan+1,2),'r-','linewidth',2);
        % plot(expand_pts(iter_scan:iter_scan+1,1),expand_pts(iter_scan:iter_scan+1,2),'g-');
    end
        
    time_plot(iter_prcd) = toc;
    
    % insert the frame to the video file. 
    if (det_mk_movie)
        frame = getframe(gcf);
        aviobj = addframe(aviobj, frame);
    end
    
    % check goal point. if so, this procedure should be terminated.
    % otherwise, procedure is proceeded.
    for idx_UAV = 1 : NUM_UAV
        if distance(HLI_pos(iter_prcd+1,(idx_UAV-1)*3+1:(idx_UAV-1)*3+2),p_f(idx_UAV,1:2)) < chk_range
            cnt_terminate(idx_UAV) = 1;
        end
        %%% Do the TASK
        if distance(HLI_pos(iter_prcd+1,(idx_UAV-1)*3+1:(idx_UAV-1)*3+2),p_task(idxMinTask(idx_UAV),:)) < chk_range
            SetTaskAllocation = [1,idx_UAV];  % event trigger!
            TASK_STATUS(1,idxMinTask(idx_UAV)) = 3; % Task is done
            iter_wpt = 2*ones(1,NUM_UAV);
            for iter = 1 : NUM_TASK
                if (TASK_STATUS(1,iter) ~= 3)
                    TASK_STATUS(:,iter) = 0; % Task is rebooted
                end
            end
        end
    end
    
    if (iter_prcd == 2000)
        break;
    end
    
    if sum(cnt_terminate) == NUM_UAV
        break;
    else
        iter_prcd = iter_prcd + 1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (det_mk_movie)
    aviobj = close(aviobj);
end

for iter_plot = 1 : NUM_UAV
    plot(HLI_pos(:,(iter_plot-1)*3+1),HLI_pos(:,(iter_plot-1)*3+2),'color',rand(1,3));
end

figure(2)
hold on;
plot(time_plot);
plot(time_cal,'color','green');

%%% Plot Results %%%
% for iter_plot = 1 : length(temp_1(:,1))
%     % from the final points,
%     if (temp_1(iter_plot,7) ~= 1) && (temp_1(iter_plot,5) ~= inf)
%         plot(temp_1(iter_plot,1),temp_1(iter_plot,2),'bo');
%         idx = iter_plot;
%         while(temp_1(idx,6)>0)
%             plot([temp_1(idx,1);temp_1(temp_1(idx,6),1)],[temp_1(idx,2);temp_1(temp_1(idx,6),2)]);
%             idx = temp_1(idx,6);
%         end
%     end
% end
% 
% for iter_plot = 1 : length(temp_2(:,1))
%     % from the final points,
%     if (temp_2(iter_plot,7) ~= 1) && (temp_2(iter_plot,5) ~= inf)
%         plot(temp_2(iter_plot,1),temp_2(iter_plot,2),'bo');        
%         idx = iter_plot;
%         while(temp_2(idx,6)>0)
%             plot([temp_2(idx,1);temp_2(temp_2(idx,6),1)],[temp_2(idx,2);temp_2(temp_2(idx,6),2)]);
%             idx = temp_2(idx,6);
%         end
%     end
% end
    
% plot(wpt(:,1),wpt(:,2),'--r','Linewidth',3)
% plot(wpt(:,1),wpt(:,2),'go')
