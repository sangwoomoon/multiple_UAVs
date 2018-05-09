function out = simulation_TA_ini(u)

    global p_task NUM_TASK NUM_UAV 
    global OBS_NUM OBS_VRT vrt_config UNOBS_NUM 
    global idxMinTask MinDistTask 
    
    iter_UAV = u(1) ; 
    Pos_XY_UAVi(1,1:2) = u(2:3) ;
    wpt(1,1:8) = u(4:11) ; 
    wpt(2,1:8) = u(12:19) ; 
    TASK_STATUS(1,1:6) = u(20:25) ; 
    TASK_STATUS(2,1:6) = u(26:31) ;
    SetTaskAllocation(1,1:2) =u(32:33) ; %  [1 0]; 

    iter_prcd=1 ; 
 
%      for iter_UAV = 1 : NUM_UAV

        HLI_pos(iter_prcd,(iter_UAV-1)*3+1:(iter_UAV-1)*3+3) = [Pos_XY_UAVi 0];
        DistPathForTask = inf*ones(NUM_UAV,NUM_TASK);
%         MinDistTask = zeros(1,NUM_UAV);
%         idxMinTask = zeros(1,NUM_UAV);
        
        if (SetTaskAllocation(1))
            for iter_TASK = 1 : NUM_TASK
                % first and second waypoints are only used.
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
   
%       end
        
        out = [iter_UAV,idxMinTask(iter_UAV),MinDistTask(iter_UAV),HLI_pos(iter_prcd,(iter_UAV-1)*3+1:(iter_UAV-1)*3+2),TASK_STATUS(1,:),TASK_STATUS(2,:),SetTaskAllocation,wpt(1,:),wpt(2,:)] ; 

end