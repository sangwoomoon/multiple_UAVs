clear all;
close all;
clc;

global iter_prcd NUM_UAV SetTaskAllocation p_i p_f p_task NUM_TASK TASK_STATUS TASK_STATUS_ini
global OBS_NUM OBS_VRT vrt_config DistPathForTask UNOBS_NUM iter_wpt cnt_terminate
global HLI_vel HLI_max_dyaw dt chk_range cnt HLI_pos second_choice wpt 
global idxMinTask MinDistTask cntt POS TASK_list SetTaskAllocation 

% 
% load Obstacles_LLH.mat

% Obstacles_LLH(:,2:3) = Obstacles_LLH(:,2:3) ;

D2R = pi/180;

scale = 1 ; 

x_ob = 17 ; 

vrt(1,1:2) = [-106.329 114.508]*scale;
vrt(2,1:2) = [55.436 -265.367]*scale;
vrt(3,1:2) = [-57.254 -305.354]*scale;
vrt(4,1:2) = [-186.302 78.156]*scale;
vrt(5,1:2) = [-106.329 114.508]*scale;
vrt(1:5,3) = 1 ; 

% vrt(6,1:2) = [40+x_ob -100]*scale;
% vrt(7,1:2) = [70+x_ob -110]*scale;
% vrt(8,1:2) = [70+x_ob -130]*scale;
% vrt(9,1:2) = [40+x_ob -140]*scale;
% vrt(10,1:2) = [40+x_ob -100]*scale;
% vrt(6:10,3) = 2 ; 
% 
% vrt(11,1:2) = [75+x_ob 25]*scale;
% vrt(12,1:2) = [100+x_ob 25]*scale;
% vrt(13,1:2) = [110+x_ob -25]*scale;
% vrt(14,1:2) = [70+x_ob -20 ]*scale;
% vrt(15,1:2) = [75+x_ob 25]*scale;
% vrt(11:15,3) = 3 ; 
% 
% vrt(16,1:2) = [5+x_ob -5]*scale;
% vrt(17,1:2) = [20+x_ob 5]*scale;
% vrt(18,1:2) = [25+x_ob -20]*scale;
% vrt(19,1:2) = [-5+x_ob -40]*scale;
% vrt(20,1:2) = [5+x_ob -5]*scale;
% vrt(16:20,3) = 4 ; 



% p_i = [48.567 -187.368;
%    136.733 -162.424 ;
%    81.442 -181.390 ]*scale;

p_i = [100 -400;
    150 -400;
    200 -400];

y_f = 60.0 ;
p_f = [-48.567 87.706+y_f ;
   10.612 100.661+y_f  ;
   -4.196 95.695+y_f ]*scale;

% 
p_task = [45 -67;
       -5 48;
       70 -40;
       65 73;
       110 -70;
       45 25]*scale;

% p_task = [25 -67;
%        -45 48;
%        70 -40;
%        50 73]*scale;
% % % 
% p_task = [37 -67;
%        70 -40]*scale;

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment and Condition Spec. %%%%%%%%%%%%%%%%%%%%%%%
% known environment.
OBS_NUM = 1;
OBS_VRT = 4;

% unknown environment.
UNOBS_NUM = 0;
UNOBS_VRT = 4;

COV_RANGE = 1; % margin to set the configuration space.
chk_range = 30; % m, required range to reach to the goal.
dt = 0.05;    % Time Interval
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Helicopter Spec. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NUM_UAV = 3;
SCAN_RANGE = 20*ones(1,NUM_UAV); %[20 20 20 20 20 20 20];
SCAN_THETA = 30*pi/180*ones(1,NUM_UAV); %[30*pi/180, 30*pi/180 30*pi/180 30*pi/180 30*pi/180 30*pi/180];

HLI_vel = ones(1,NUM_UAV)*1; %[1 1 1 1 1 1]; % m/s
HLI_max_dyaw = 20*pi/180*ones(1,NUM_UAV); %[0.4 0.4 0.4 0.4 0.4 0.4]; %rad/s
cnt = 0 ; 
cntt = 0 ; 
% cnt(1:NUM_UAV) = chk_range / dt+3; % count to prceed to the next step.
cnt_terminate = zeros(1,NUM_UAV);  % binary value for mission requirement
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Task Spec.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
NUM_TASK = 6 ;
TASK_TIME = 1; % second
TASK_STATUS = zeros(2,NUM_TASK); % 0:Not yet, 1: Scheduled, 2:Doing 3:Done // 2nd tag::UAV#
TASK_STATUS_prev = zeros(2,NUM_TASK) ; 
SetTaskAllocation = [1,0];  % event trigger (1 : TA OPERATION), 2nd col:triggered UAV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL CONDITION SETTEING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% axis([-400 200 -100 400])
% set(gcf,'color',[1 1 1]);
% grid on
% hold on
% axis equal tight

fig = figure(1);
set(fig,'DoubleBuffer','on');
hold on;

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
grid on;



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

Pos_XY_UAV(1,:) = p_i(1,1:2);
Pos_XY_UAV(2,:) = p_i(2,1:2); 
Pos_XY_UAV(3,:) = p_i(3,1:2); 

    if (iter_prcd == 1)
        for iter_UAV = 1 : NUM_UAV
            HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3) = [p_i(iter_UAV,:) 0];
            wpt(1:2,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(2,4);
            second_choice(1,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(1,4);
        end
    end
    
 HLI_pos(iter_prcd,1:3) = [Pos_XY_UAV(1,:) 0 ];
 HLI_pos(iter_prcd,4:6) = [Pos_XY_UAV(2,:) 0 ];
 HLI_pos(iter_prcd,7:9) = [Pos_XY_UAV(3,:) 0 ];
 

 
