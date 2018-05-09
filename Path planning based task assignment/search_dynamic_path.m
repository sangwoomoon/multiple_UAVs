function [wpt] = search_dynamic_path(SCAN_RANGE,SCAN_THETA,HLI_vel,HLI_max_dyaw,dt,OBS_NUM,OBS_VRT,vrt,points,angle)

% Initial
wpt = [points(1,1), points(1,2), angle];
p_f = points(2,:);

%%% 해야할 일
%%% scanning 할 때 꼭지점 부분 해결 (대강 해결됨)
%%% 속도가 조건에 따라 변하는 거 적용.

det_tot = 0;
get_first = 0;
landmark_prev = zeros(2,3);
i = 1;
while (1)
    
    % 장애물 스캔.
    [meet_pts,flag_meet_pts] = scan_obstacle(wpt(i,:),vrt,OBS_NUM,OBS_VRT,SCAN_RANGE,SCAN_THETA);
    
    if (flag_meet_pts)
        % landmark setting for scanned points.        
        [landmark, get_first] = set_landmark(meet_pts,get_first,COV_RANGE);
        
        % 처음 landmark를 tot에 할당.
        if (get_first == 1)
            % 모서리가 아닌 포인트로 할당 받을 때 문제점 발생 가능성 존재 (거의 희박)
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
                                    landmark_tot(iter_tot,:) = landmark(iter_landmark+iter_update-1,:);
                                else
                                    landmark_tot(iter_tot+1,:) = landmark(iter_landmark+iter_update-1,:);                                    
                                end
                            end
                        end
                        det_new_edge = 0;
                    end
                end
                if det_new_edge == 1   %%% 새로운 모서리이다
                    landmark_tot(length(landmark_tot(:,1))+1:length(landmark_tot(:,1))+2,:) = landmark(iter_landmark:iter_landmark+1,:);
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
        line(wpt(:,1),wpt(:,2),'color','black');         
    else
        [wpt(i+1,:),landmark_prev] = find_dynamic_path(wpt(i,:),landmark_config,wpt(i,:),p_f,HLI_vel,dt,HLI_max_dyaw,landmark_prev);
        line(wpt(:,1),wpt(:,2),'color','black');        
    end
    
    % 끝에 도달하면 종료.
    if distance(wpt(i+1,:),p_f) < HLI_vel*dt
        break;
    end
    
    clear meet_pts;
    i = i + 1;
end