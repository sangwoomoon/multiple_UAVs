
%%% Vertex Database Type %%%
%%% [ x y idx_bldg idx_edge invite determinent_bldg invite determinent_edge] %%%

%%% Waypoint Database Type %%%
%%% [ x y idx_bldg idx_edge] %%%


%%%  TO DO %%%
%%% Dynamic path 를 구할 때 cover range 안에 들어가는 거 처리
%%% CUL-DE-SAC 처리




clear all;
% close all;
clc;



%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment and Condition Spec. %%%%%%%%%%%%%%%%%%%%%%%
% known environment.
OBS_NUM = 2;
OBS_VRT = 4;

% unknown environment.
UNOBS_NUM = 2;
UNOBS_VRT = 4;

COV_RANGE = 2; % margin to set the configuration space.
chk_range = 2; % m, required range to reach to the goal.
dt = 0.2;    % Time Interval
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Helicopter Spec. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NUM_UAV = 3;
SCAN_RANGE = 20*ones(1,NUM_UAV);
SCAN_THETA = 30*pi/180*ones(1,NUM_UAV); 

HLI_vel = ones(1,NUM_UAV); % m/s
HLI_max_dyaw = 0.4*ones(1,NUM_UAV);  %rad/s
cnt(1:NUM_UAV) = chk_range / dt+3; % count to prceed to the next step.
cnt_terminate = zeros(1,NUM_UAV);  % binary value for mission requirement
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Movie File Spec. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
det_mk_movie = 1; % 0 : do not make, 1 : make
if (det_mk_movie)
    frame_per_sec = 10; % fps. 15 is default in Matlab
    file_name = 'movie_file.avi';
    codeck = 'IVUY'; % compressing codeck
    quality = 100;  % quality of file. 100 is maximum.
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL CONDITION SETTEING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
axis([-50 50 -50 50])
set(gcf,'color',[1 1 1]);
grid on
hold on
axis equal tight

fig = figure(1);
set(fig,'DoubleBuffer','on');

if (OBS_NUM == 0)
    % title('Realtime Path Planning in Totally Unknown Environment'); %, Sangwoo Moon, 2010');
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
    %p_i(iter,1:2) = [-60 40]+[120*rand(1) 10*rand(1)];
     [p_i(iter,1), p_i(iter,2)] = ginput(1);% Initial Point
    plot(p_i(iter,1),p_i(iter,2), 'ro');
    text(p_i(iter,1)-5,p_i(iter,2)-2,sprintf('%s %d %s','UAV',iter,'Start'));

    %p_f(iter,1:2) = [60 -40]+[-120*rand(1) -10*rand(1)];
     [p_f(iter,1), p_f(iter,2)] = ginput(1);% Final Point
    plot(p_f(iter,1),p_f(iter,2), 'bo');
    text(p_f(iter,1),p_f(iter,2)-2,sprintf('%s %d %s','UAV',iter,'Goal'));
end

% [-60 60 40 50]
% [-60 60 -40 -50]


% Known Obstacles setting and calculate Configuration Space for known obstacles
if (OBS_NUM ~= 0)
    vrt = set_obstacle(OBS_NUM,OBS_VRT,0,1);
    vrt_config = set_config_space(vrt,OBS_NUM,OBS_VRT,COV_RANGE);
elseif (OBS_NUM == 0)
    vrt_config = zeros(4,6);
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ANIMATION SETTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helicopter scale.
scale = 0.15;

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
   aviobj = avifile(file_name,'fps',frame_per_sec,'Quality',quality); %'compression',codek (oem)
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
while (1)
    
    tic;
    
    for iter_UAV = 1 : NUM_UAV
        if (iter_prcd == 1)
            wpt(1,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(1,4);
            second_choice(1,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(1,4);
            % first and second waypoints are olny used.
            [wpt_temp, second_choice_temp, temp_1] = operate_algorithm(OBS_NUM,OBS_VRT,wpt((iter_UAV-1)*4+1:iter_UAV*4),vrt_config,vrt_config,p_i(iter_UAV,:),p_f(iter_UAV,:),1);
            wpt(1:length(wpt_temp(:,1)),(iter_UAV-1)*4+1:(iter_UAV*4)) = wpt_temp;
            second_choice(1,(iter_UAV-1)*4+1:iter_UAV*4) = second_choice_temp;
            HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3) = [wpt(1,(iter_UAV-1)*4+1:(iter_UAV-1)*4+2) atan2(wpt(2,(iter_UAV-1)*4+2)-wpt(1,(iter_UAV-1)*4+2),wpt(2,(iter_UAV-1)*4+1)-wpt(1,(iter_UAV-1)*4+1))];
            
            clear wpt_temp second_choice_temp;
        end

        % calculate the UAV position
        if (cnt_terminate(iter_UAV) == 0)
            [HLI_pos(iter_prcd+1,(iter_UAV-1)*3+1:iter_UAV*3),flag_change] = ...
                calculate_postion(HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3),wpt(iter_wpt(iter_UAV),(iter_UAV-1)*4+1:(iter_UAV-1)*4+2),HLI_vel(iter_UAV),HLI_max_dyaw(iter_UAV),dt,chk_range);
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
        if (flag_meet_pts == 1)
            [wpt_temp, second_choice_temp, temp_1] = ...
                operate_algorithm(OBS_NUM,OBS_VRT,wpt(:,(iter_UAV-1)*4+1:(iter_UAV-1)*4+2),...
                [vrt_config;expand_pts(:,1:4),zeros(length(expand_pts(:,1)),2)],[vrt_config;meet_pts(:,1:4),zeros(length(meet_pts(:,1)),2)],...
                HLI_pos(iter_prcd+1,(iter_UAV-1)*3+1:(iter_UAV-1)*3+2),p_f(iter_UAV,1:2),iter_wpt(iter_UAV)-1);
             
            wpt(1:length(wpt_temp(:,1)),(iter_UAV-1)*4+1:(iter_UAV*4)) = wpt_temp;
            second_choice(1,(iter_UAV-1)*4+1:iter_UAV*4) = second_choice_temp;

%             if (iter_UAV == 1)
%                 wpt(1:length(wpt_temp(:,1)),(iter_UAV-1)*4+1:(iter_UAV*4)) = wpt_temp;
%             elseif (iter_UAV == 2)
%                 wpt(1:length(second_choice_temp(:,1)),(iter_UAV-1)*4+1:(iter_UAV*4)) = second_choice_temp;
%             end
            
            clear wpt_temp second_choice_temp;
        end
    end

    time_cal(iter_prcd) = toc;
  
    for idx_UAV = 1 : NUM_UAV
        POS_UAV_X(idx_UAV) = HLI_pos(iter_prcd,3*(idx_UAV-1)+1);
        POS_UAV_Y(idx_UAV) = HLI_pos(iter_prcd,3*(idx_UAV-1)+2);
    end
        
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
        set(handle_waypoint(idx_UAV),'XData',wpt(iter_wpt(idx_UAV),(idx_UAV-1)*4+1),'YData',wpt(iter_wpt(idx_UAV),(idx_UAV-1)*4+2),'Color',index_clr((idx_UAV-1)*3+1:idx_UAV*3));
        % set(handle_second_wpt(idx_UAV),'XData',second_choice(1,(idx_UAV-1)*4+1),'YData',second_choice(1,(idx_UAV-1)*4+2),'Color',index_clr((idx_UAV-1)*3+1:idx_UAV*3));
        if (UNOBS_NUM ~= 0)
            set(handle_srh(idx_UAV),'Xdata',heli_srh_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_srh_trans(:,2)+POS_UAV_Y(idx_UAV));
        end
        plot(HLI_pos([iter_prcd,iter_prcd+1],(idx_UAV-1)*3+1),HLI_pos([iter_prcd,iter_prcd+1],(idx_UAV-1)*3+2),'k-');
    end
    drawnow; 
    
     for iter_scan = 1 : 2 : length(meet_pts(:,1))-1
         plot(search_pts(iter_scan:iter_scan+1,1),search_pts(iter_scan:iter_scan+1,2),'r-','linewidth',3);
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
    end
           
    if sum(cnt_terminate) == NUM_UAV
        break;
    else
        iter_prcd = iter_prcd + 1;
    end
    
    if mod(iter_prcd,200) == 0
        iter_prcd;
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (det_mk_movie)
    aviobj = close(aviobj);
end

figure(2)
oper_time = 0 : dt : (iter_prcd-1)*dt;
hold on;
% plot(oper_time,time_plot);
plot(oper_time,time_cal,'ro--');

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
    
%  plot(wpt(:,1),wpt(:,2),'--r','Linewidth',3)
% plot(wpt(:,1),wpt(:,2),'go')
