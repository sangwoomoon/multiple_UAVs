% function out = popup_obstacles(u)

global dt p_i p_f p_task NUM_UAV NUM_TASK cnt chk_range
global wpt_list task_list wpt popup 


% 
% load Obstacles_LLH.mat

% Obstacles_LLH(:,2:3) = Obstacles_LLH(:,2:3) ;

D2R = pi/180;

scale = 1 ; 

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

vrt(26:35,1:2) = [-10000 12000;
    -10700 14200;
    -12500 14700;
    -11900 12400
    -10000 12000;
    -4500 11000;
    -3800 14000;
    -7500 13000;
    -6000 10000;
    -4500 11000];

vrt(26:30,3) = 6 ; 
vrt(31:35,3) = 7 ; 

p_i = [-10000.0 0;
     -4800 1000;
-1083.6933276395785 1527.1942766592483];

p_task = [-12842.74 12076.025;   
-10584.675 16169.59;
-7197.5800 14415.205;
-1713.71 15935.6750;
-1000 14000
-2681.45 12543.86;
vrt(28,1:2);
-5000 16000;
1308.0787930389083 16373.925361067799];

p_f = [-9000 18000;
-5000 16000;
1308.0787930389083 16373.925361067799];




%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment and Condition Spec. %%%%%%%%%%%%%%%%%%%%%%%
% known environment.
OBS_NUM = 7;
OBS_VRT = 4;

% unknown environment.
UNOBS_NUM = 0;
UNOBS_VRT = 4;

NUM_TASK = 9 ; 
popup = 1 ; 
% COV_RANGE = 1; % margin to set the configuration space.
% chk_range = 15; % m, required range to reach to the goal.
dt = 0.1;    % Time Interval
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL CONDITION SETTEING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% axis([-400 200 -100 400])
% set(gcf,'color',[1 1 1]);
% grid on
% hold on
% axis equal tight

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

    wpt_list = zeros(NUM_TASK,NUM_UAV*2) ;
    task_list = zeros(NUM_TASK,NUM_UAV) ; 
    wpt = zeros(1,NUM_UAV*2) ; 
    
    
task_list(1:5,1) = [1 7 2 3 8]';
task_list(1:1,2) = [8]';
task_list(1:4,3) = [6 5 4 9]';
    
    for iduav = 1 : NUM_UAV
       for idtask = 1 : NUM_TASK
           if (task_list(idtask,iduav) ~= 0)
              wpt_list(idtask,(iduav-1)*2+1:iduav*2) = p_task(task_list(idtask,iduav),:);
           else 
              wpt_list(idtask,(iduav-1)*2+1:iduav*2) = p_f(iduav,:);
           end
       end
    end

        wpt = wpt_list(1,:);
