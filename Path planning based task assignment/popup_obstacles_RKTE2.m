% function out = popup_obstacles(u)

global SetTaskAllocation
global OBS_NUM OBS_VRT vrt_config UNOBS_NUM 


% 
% load Obstacles_LLH.mat

% Obstacles_LLH(:,2:3) = Obstacles_LLH(:,2:3) ;

D2R = pi/180;

scale = 1 ; 

vrt(1,1:2) = [-106.329 114.508]*scale;
vrt(2,1:2) = [55.436 -265.367]*scale;
vrt(3,1:2) = [-57.254 -305.354]*scale;
vrt(4,1:2) = [-186.302 78.156]*scale;
vrt(5,1:2) = [-106.329 114.508]*scale;
vrt(1:5,3) = 1 ; 

vrt(6,1:2) = [-10 0]*scale;
vrt(7,1:2) = [90 30]*scale;
vrt(8,1:2) = [60 100]*scale;
vrt(9,1:2) = [-25 75]*scale;
vrt(10,1:2) = [-10 0]*scale;
vrt(6:10,3) = 2 ; 




%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment and Condition Spec. %%%%%%%%%%%%%%%%%%%%%%%
% known environment.
OBS_NUM = 2;
OBS_VRT = 4;

% unknown environment.
UNOBS_NUM = 0;
UNOBS_VRT = 4;

COV_RANGE = 1; % margin to set the configuration space.
chk_range = 3; % m, required range to reach to the goal.
dt = 0.05;    % Time Interval
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

SetTaskAllocation(1) = 1 ; 


% out = [


% end