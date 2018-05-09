
clear all;
load('replay_data.mat');
clc;
clf;

SCAN_RANGE = 20;
SCAN_THETA = 30*pi/180;
del_theta = 0.02;

HLI_vel = 5; % m/s
HLI_max_dyaw = 1; %rad/s
dt = 0.2;    % Time Interval

COV_RANGE = 3;
OBS_NUM = 20;
OBS_VRT = 4;

figure(1)
axis([0 200 0 200])
grid on
hold on
axis equal tight

% [p_i(1,1), p_i(1,2)] = ginput(1);% Initial Point
% plot(p_i(:,1),p_i(:,2), 'ro');
% text(p_i(:,1)-15,p_i(:,2)-5,sprintf('%s','Starting Point'));

% [p_f(1,1), p_f(1,2)] = ginput(1);% Final Point
% plot(p_f(:,1),p_f(:,2), 'bo');
% text(p_f(:,1)-15,p_f(:,2)-5,sprintf('%s','Goal Point'));

% Obstacles
% vrt = set_obstacle(OBS_NUM,OBS_VRT);

% Update current location of Helicopter

% Initial
x = p_i(1,1);
y = p_i(1,2);
p_f = p_pass;
angle = 0.1; %atan2(p_f(1,2)-p_i(1,2),p_f(1,1)-p_i(1,1));

for iter = 1 : OBS_VRT+1 : length(vrt(:,1))-OBS_VRT+1
    patch(vrt(iter:iter+OBS_VRT,1),vrt(iter:iter+OBS_VRT,2),'green');
end

wpt = [x, y, angle];

%%% 해야할 일
%%% scanning 할 때 꼭지점 부분 해결 (대강 해결됨)
%%% 두 변을 동시에 스캔할 때 의사결정을 못함 (edge 리뉴얼 문제)
%%% 점으로 스캔 될 경우의 landmark 생성 문제.
%%% 속도가 조건에 따라 변하는 거 적용.

det_tot = 0;
get_first = 0;
landmark_prev = zeros(2,3);
i = 1;
while (1)
    
    % 장애물 스캔.
    [meet_pts,flag_meet_pts] = scan_obstacle(wpt(i,:),vrt,OBS_NUM,OBS_VRT,SCAN_RANGE,SCAN_THETA,del_theta);
    
    if (flag_meet_pts)
        % landmark setting for scanned points.        
        [landmark, get_first] = set_landmark(meet_pts,get_first,COV_RANGE);
        
        % 처음 landmark를 tot에 할당.
        if (get_first == 1)
            idx_ldmk = 1;
            for iter = 1 : 2 : length(landmark(:,1))-1
                if (landmark(iter,1) ~= landmark(iter+1,1)) || (landmark(iter,2) ~= landmark(iter+1,2))
                    landmark_tot([idx_ldmk,idx_ldmk+1],:) = landmark([iter,iter+1],:);
                    idx_ldmk = idx_ldmk + 2;
                end
            end
            det_tot = 1;
        end

        % landmark 전체 통합. (line-line 통합)            
        if (get_first >= 2)
            det_new_edge = 1;
            for iter_landmark = 1 : 2 : length(landmark(:,1))-1
                for iter_tot = 1 : 2 : length(landmark_tot(:,1))-1
                    if (check_new_edge(landmark([iter_landmark,iter_landmark+1],:),landmark_tot([iter_tot,iter_tot+1],:)) ~= 1) %%% 새로운 모서리가 아니다
                        for iter_update = 1 : 2
                            if (determine_on_line(landmark(iter_landmark+iter_update-1,:),landmark_tot(iter_tot,:),landmark_tot(iter_tot+1,:)) == 0)
                                if (distance(landmark(iter_landmark+iter_update-1,:),landmark_tot(iter_tot,:)) < distance(landmark(iter_landmark+iter_update-1,:),landmark_tot(iter_tot+1,:)))
                                    landmark_tot(iter_tot,1:2) = landmark(iter_landmark+iter_update-1,:);
                                else
                                    landmark_tot(iter_tot+1,1:2) = landmark(iter_landmark+iter_update-1,:);                                    
                                end
                            end
                        end
                        det_new_edge = 0;
                    end
                end
                if det_new_edge == 1   %%% 새로운 모서리이다
                    landmark_tot(length(landmark_tot(:,1))+1:length(landmark_tot(:,1))+2,1:2) = landmark(iter_landmark:iter_landmark+1,:);
                end
            end
        end
        
        %%% landmark 주변에 인접한 line이 존재하는지 확인하기
        if (length(landmark(:,1)) >= 3)
            landmark_tot(:,3) = 0;            
            for chk_i = 1 : 2 : length(landmark_tot(:,1))-2
                for chk_j = 3 : 2 : length(landmark_tot(:,1))-1
                    for chk_k = 1 : 2
                        line_slp = (landmark_tot(chk_j,2)-landmark_tot(chk_j+1,2))/(landmark_tot(chk_j,1)-landmark_tot(chk_j+1,1));
                        dist_pt_line = abs(line_slp*landmark_tot(chk_i+chk_k-1,1)-landmark_tot(chk_i+chk_k-1,2)-line_slp*landmark_tot(chk_j,1)+landmark_tot(chk_j,2))/sqrt(line_slp^2+1);
                        if (dist_pt_line < 2*COV_RANGE) %%% 점과 line 사이의 거리가 Cover Range 보다 작으면 marking을 하여 sink point에 빠지지 않게 한다.
                            landmark_tot(chk_i+chk_k-1,3) = landmark_tot(chk_i+chk_k-1,3) + 1; % Marking.
                        end
                    end
                end
            end
        end
        
        %%%%%%%%%%%%%% 여기까지 LANDMARK TOTAL 만드는 함수 생성하기.

        % configuration space 만들기.
        if (det_tot)
            for idx = 1 : 2 : length(landmark_tot(:,1))-1
                edge_slp_angle = -atan2(landmark_tot(idx+1,2)-landmark_tot(idx,2),landmark_tot(idx+1,1)-landmark_tot(idx,1));
                det_where_wpt = sin(edge_slp_angle)*(wpt(i,1)-landmark_tot(idx,1))+cos(edge_slp_angle)*(wpt(i,2)-landmark_tot(idx,2));
                % singular 경우를 막기 위한 미봉책.
                if abs(landmark_tot(idx+1,1) - landmark_tot(idx,1)) < 0.00000001
                    landmark_tot(idx+1,1) = landmark_tot(idx+1,1) + 0.0001;
                end
                a = (landmark_tot(idx+1,2)-landmark_tot(idx,2))/(landmark_tot(idx+1,1)-landmark_tot(idx,1));
                if abs(a) < 0.00001
                    a = 0.00001;
                end
                for idx_ldmk = 1 : 2
                    if (det_where_wpt < 0)
                        landmark_config(idx+idx_ldmk-1,1) = -a/(1+a^2)*(-a*landmark_tot(idx+idx_ldmk-1,1)+landmark_tot(idx+idx_ldmk-1,2)-COV_RANGE/cos(-edge_slp_angle)-1/a*landmark_tot(idx+idx_ldmk-1,1)-landmark_tot(idx+idx_ldmk-1,2));
                        landmark_config(idx+idx_ldmk-1,2) = -a/(1+a^2)*(landmark_tot(idx+idx_ldmk-1,1)-1/a*landmark_tot(idx+idx_ldmk-1,2)+COV_RANGE/(a*cos(-edge_slp_angle))-landmark_tot(idx+idx_ldmk-1,1)-a*landmark_tot(idx+idx_ldmk-1,2));
                    else
                        landmark_config(idx+idx_ldmk-1,1) = -a/(1+a^2)*(-a*landmark_tot(idx+idx_ldmk-1,1)+landmark_tot(idx+idx_ldmk-1,2)+COV_RANGE/cos(-edge_slp_angle)-1/a*landmark_tot(idx+idx_ldmk-1,1)-landmark_tot(idx+idx_ldmk-1,2));
                        landmark_config(idx+idx_ldmk-1,2) = -a/(1+a^2)*(landmark_tot(idx+idx_ldmk-1,1)-1/a*landmark_tot(idx+idx_ldmk-1,2)-COV_RANGE/(a*cos(-edge_slp_angle))-landmark_tot(idx+idx_ldmk-1,1)-a*landmark_tot(idx+idx_ldmk-1,2));
                    end
                end
                % configration 좌우 영역 넓히기
                theta = atan2(landmark_config(idx+1,2)-landmark_config(idx,2),landmark_config(idx+1,1)-landmark_config(idx,1));
                landmark_xtd(1,:) = [landmark_config(idx,1)-COV_RANGE*cos(theta),landmark_config(idx,2)-COV_RANGE*sin(theta)];
                landmark_xtd(2,:) = [landmark_config(idx+1,1)+COV_RANGE*cos(theta),landmark_config(idx+1,2)+COV_RANGE*sin(theta)];
                if (distance(landmark_xtd(1,:),landmark_xtd(2,:)) < distance(landmark_config(idx+1,:),landmark_config(idx,:)))
                    landmark_config(idx,:) = [landmark_config(idx,1)+COV_RANGE*cos(theta),landmark_config(idx,2)+COV_RANGE*sin(theta)];
                    landmark_config(idx+1,:) = [landmark_config(idx+1,1)-COV_RANGE*cos(theta),landmark_config(idx+1,2)-COV_RANGE*sin(theta)];                    
                else
                    landmark_config([idx,idx+1],:) = landmark_xtd;
                end
            end
            for iter = 1 : 2 : length(landmark_config(:,1))-1
                plot(landmark_tot([iter,iter+1],1),landmark_tot([iter,iter+1],2),'Color',rand(1,3),'LineWidth',3);            
                plot(landmark_config([iter,iter+1],1),landmark_config([iter,iter+1],2),'Color',rand(1,3),'LineWidth',2);
            end     
        end
       
        clear landmark;         
    end   
    
    if (flag_meet_pts == 0) && (det_tot == 0)
        theta = atan2(p_f(1,2)-wpt(i,2),p_f(1,1)-wpt(i,1));
        if abs(theta-wpt(i,3)) > HLI_max_dyaw*dt
            if (theta-wpt(i,3) > HLI_max_dyaw*dt)
                theta = HLI_max_dyaw*dt+wpt(i,3);
            elseif (theta-wpt(i,3) < -HLI_max_dyaw*dt)
                theta = -HLI_max_dyaw*dt+wpt(i,3);
            end
        end                
        wpt(i+1,:) = [wpt(i,1)+HLI_vel*dt*cos(theta),wpt(i,2)+HLI_vel*dt*sin(theta),theta];
        line(wpt(:,1),wpt(:,2),'color','black','LineWidth',3);         
    else
        [wpt(i+1,:),landmark_prev] = find_dynamic_path(wpt(i,:),landmark_config,wpt(i,:),p_f,HLI_vel,dt,HLI_max_dyaw,landmark_prev,angle);
        line(wpt(:,1),wpt(:,2),'color','black','LineWidth',3);        
    end
    
        if i == 10
            angle = atan2(18.81-wpt(10,2),-38.48-wpt(10,1));
        elseif i == 17
            angle = atan2(20.86-wpt(17,2),-38.45-wpt(17,1));
        elseif i == 28
            angle = atan2(24.37-wpt(28,2),-25.49-wpt(28,1));
        elseif i == 33
            angle = atan2(24.37-wpt(33,2),-25.49-wpt(33,1));
        elseif i == 43            
            angle = atan2(23.92-wpt(43,2),-22.26-wpt(43,1));
        elseif i == 48            
            angle = atan2(25.88-wpt(48,2),-0.05-wpt(48,1));
        elseif i == 69
            angle = atan2(p_f(1,2)-wpt(69,2),p_f(1,1)-wpt(69,1));
        elseif i == 74            
            angle = atan2(36.19-wpt(74,2),1.964-wpt(74,1));
        elseif i == 78        
            angle = atan2(36.17-wpt(78,2),0-wpt(78,1));
        elseif i == 85
            angle = atan2(p_f(1,2)-wpt(85,2),p_f(1,1)-wpt(85,1));
        elseif i == 89        
            angle = atan2(56.5-wpt(89,2),4-wpt(89,1));
        end 
        
        if i > 89
            angle;
        end
    
    % 끝에 도달하면 종료.
    if distance(wpt(i+1,:),p_f) < 1
        break;
    end
    
    clear meet_pts;
    i = i + 1;
end

line(wpt(:,1),wpt(:,2),'color','red','LineWidth',3);
for iter = 1 : 2 : length(landmark_tot(:,1))-1
    plot(landmark_tot([iter,iter+1],1),landmark_tot([iter,iter+1],2),'color','blue','LineWidth',4);
end

