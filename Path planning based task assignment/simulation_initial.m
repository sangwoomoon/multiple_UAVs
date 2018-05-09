clear all;
close all;
clc;

global iter_prcd NUM_UAV SetTaskAllocation p_i p_f p_task NUM_TASK TASK_STATUS TASK_STATUS_ini
global OBS_NUM OBS_VRT vrt_config DistPathForTask UNOBS_NUM iter_wpt cnt_terminate
global HLI_vel HLI_max_dyaw dt chk_range cnt HLI_pos second_choice wpt 
global idxMinTask_ini MinDistTask_ini  ini_HLI_posi 


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

obstacle_oem = [NED(:,2) NED(:,1)] ;
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

% p_i(1:2,1:2) = obstacle_oem(21:22,:);
p_f(1:2,1:2) = obstacle_oem(23:24,:);

p_i(1,:) = [-150 -125]*50; 
p_i(2,:) = [-200 -125]*50; 
p_i(3,:) = [-100 -125]*50;
p_f(3,:) = [-100 300]*50;
%     p_f=[61 -100 ; 
%         118 -102 ;
%         132 -58]*20;
%         
%     p_i = [-145 557;
%         -99 575;
%         -64 594]*20;
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment and Condition Spec. %%%%%%%%%%%%%%%%%%%%%%%
% known environment.
OBS_NUM = 5;
OBS_VRT = 4;

% unknown environment.
UNOBS_NUM = 0;
UNOBS_VRT = 4;

COV_RANGE = 3; % margin to set the configuration space.
chk_range = 150; % m, required range to reach to the goal.
dt = 0.08;    % Time Interval
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
TASK_STATUS_ini = zeros(2,NUM_TASK) ; 
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

% p_i = [ -100 50 ; 25 50 ; 100 50 ] ; 
% p_f = [-50 250 ; 0 250 ; 50 250 ] ;
p_task = [-256.8548 241.5205 ;
    -211.6935 323.3918;
    -143.9516 288.3041;
    -6.8548 200.5848;
    -53.6290 250.8772;
    -34.2742 318.7135]*50; 
% p_task = [ 1 1 ;
%         80 84;
%         -30 124]*20;
    %        20 197;         -73 258;        -25 333]*20;

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
 
 
 ini_iter_UAV = 1 ;  
 ini_HLI_posi =  HLI_pos(iter_prcd,:) ;
 ini_idxMinTask = [1 2 3] ;
 ini_MinDistTask = [1e6 1e6 1e6] ;
 idxMinTask_ini = [1 2 3] ; 
 MinDistTask_ini = [1e6 1e6 1e6] ; 
 sum_done = 0 ; 
