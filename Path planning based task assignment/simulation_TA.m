function out = simulation_TA(Pos_XY_UAV1,Pos_XY_UAV2)
    
    global iter_prcd NUM_UAV SetTaskAllocation p_i p_f p_task NUM_TASK TASK_STATUS
    global OBS_NUM OBS_VRT vrt_config DistPathForTask UNOBS_NUM iter_wpt cnt_terminate
    global HLI_vel HLI_max_dyaw dt chk_range cnt HLI_pos second_choice wpt

    
    
 

    tic;

           HLI_pos(iter_prcd,1:3) = [Pos_XY_UAV1 0 ];
           HLI_pos(iter_prcd,4:6) = [Pos_XY_UAV2 0 ];
    

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
    
%     tic;
%     
%     for idx_UAV = 1 : NUM_UAV
%         % Angle Assign 
%         euler = [cos(HLI_pos(iter_prcd,3*(idx_UAV-1)+3)),-sin(HLI_pos(iter_prcd,3*(idx_UAV-1)+3));sin(HLI_pos(iter_prcd,3*(idx_UAV-1)+3)),cos(HLI_pos(iter_prcd,3*(idx_UAV-1)+3))];
%         heli_head_trans=[heli_head(:,1) heli_head(:,2)]*euler';
%         heli_boom_trans = [heli_boom(:,1) heli_boom(:,2)]*euler';
%         heli_tail_trans = [heli_tail(:,1) heli_tail(:,2)]*euler';
%         heli_rotor_trans = [heli_rotor(:,1) heli_rotor(:,2)]*euler';
%         heli_srh_trans = [set_bnd(:,1),set_bnd(:,2)]*euler';
%         
%         set(head(idx_UAV),'Xdata',heli_head_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_head_trans(:,2)+POS_UAV_Y(idx_UAV));
%         set(rotor(idx_UAV),'Xdata',heli_rotor_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_rotor_trans(:,2)+POS_UAV_Y(idx_UAV));
%         set(boom(idx_UAV),'Xdata',heli_boom_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_boom_trans(:,2)+POS_UAV_Y(idx_UAV));
%         set(tail(idx_UAV),'Xdata',heli_tail_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_tail_trans(:,2)+POS_UAV_Y(idx_UAV));
% %         set(handle_waypoint(idx_UAV),'XData',wpt(iter_wpt(idx_UAV),(idx_UAV-1)*4+1),'YData',wpt(iter_wpt(idx_UAV),(idx_UAV-1)*4+2),'Color',index_clr((idx_UAV-1)*3+1:idx_UAV*3));
%         % set(handle_second_wpt(idx_UAV),'XData',second_choice(1,(idx_UAV-1)*4+1),'YData',second_choice(1,(idx_UAV-1)*4+2),'Color',index_clr((idx_UAV-1)*3+1:idx_UAV*3));
%         if (UNOBS_NUM ~= 0)
%             set(handle_srh(idx_UAV),'Xdata',heli_srh_trans(:,1)+POS_UAV_X(idx_UAV),'YData',heli_srh_trans(:,2)+POS_UAV_Y(idx_UAV));
%         end
%         % plot(HLI_pos([iter_prcd,iter_prcd+1],(idx_UAV-1)*3+1),HLI_pos([iter_prcd,iter_prcd+1],(idx_UAV-1)*3+2),'k-');
%     end
%     drawnow; 
%     
%     for iter_scan = 1 : 2 : length(meet_pts(:,1))-1
%         plot(search_pts(iter_scan:iter_scan+1,1),search_pts(iter_scan:iter_scan+1,2),'r-','linewidth',2);
%         % plot(meet_pts(iter_scan:iter_scan+1,1),meet_pts(iter_scan:iter_scan+1,2),'r-','linewidth',2);
%         % plot(expand_pts(iter_scan:iter_scan+1,1),expand_pts(iter_scan:iter_scan+1,2),'g-');
% %     end
%         
%     time_plot(iter_prcd) = toc;
    
    % insert the frame to the video file. 
%     if (det_mk_movie)
%         frame = getframe(gcf);
%         aviobj = addframe(aviobj, frame);
%     end
    
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
    
%     if (iter_prcd == 2000)
%         break;
%     end
%     
%     if sum(cnt_terminate) == NUM_UAV
%         break;
%     else
%         iter_prcd = iter_prcd + 1;
%     end

Pos_XY_Ref_UAV1 = wpt(2,1:2);
Pos_XY_Ref_UAV2 = wpt(2,5:6);

Pos_UAV1 = [Pos_XY_UAV1 ; Pos_XY_Ref_UAV1];
Pos_UAV2 = [Pos_XY_UAV2 ; Pos_XY_Ref_UAV2];

out =[Pos_UAV1 Pos_UAV2] ;

end 
