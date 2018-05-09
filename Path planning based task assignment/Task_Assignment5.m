function out = Task_Assignment5(u) 

global iter_prcd NUM_UAV SetTaskAllocation p_i p_f p_task NUM_TASK TASK_STATUS 
global OBS_NUM OBS_VRT vrt_config DistPathForTask iter_wpt cnt_terminate
global chk_range HLI_pos second_choice wpt cnt 
global idxMinTask MinDistTask POS cntt TASK_list TASK_STATUS_prev

meet_pts = zeros(1,6);  % searching points 
scan_point = zeros(1,6); 
discon = zeros(1,NUM_UAV);



iter_prcd = 1 ;

% iter_wpt(1:NUM_UAV) = 2;
% DistPathForTask = inf*ones(NUM_UAV,NUM_TASK);
% MinDistTask = zeros(1,NUM_UAV);
% idxMinTask = zeros(1,NUM_UAV);

clock_zero = u(NUM_UAV*2+1) ; 

    if (clock_zero == 0 )
        for iter_UAV = 1 : NUM_UAV
            HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3) = [p_i(iter_UAV,:) 0];
            wpt(1,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(1,4);
            second_choice(1,(iter_UAV-1)*4+1:iter_UAV*4) = zeros(1,4);
            SetTaskAllocation = [1 1] ;
        end
    end
    
    
    for idx_UAV = 1 : NUM_UAV
           HLI_pos(iter_prcd,3*(idx_UAV-1)+1) = u((idx_UAV-1)*2+1) ;
           HLI_pos(iter_prcd,3*(idx_UAV-1)+2) = u((idx_UAV-1)*2+2) ;
        
%         if (idxMinTask(idx_UAV) == 0 )
%             idxMinTask(idx_UAV) = 1 ; 
%         elseif idx_UAV ~= 3         % agent 3 is not connected
%             SetTaskAllocation(1) = 1 ;
%         end
        
        if(HLI_pos(iter_prcd,3*(idx_UAV-1)+1) ==0 && HLI_pos(iter_prcd,3*(idx_UAV-1)+2)==0)
             HLI_pos(iter_prcd,(idx_UAV-1)*3+1:(idx_UAV-1)*3+3) = [p_i(idx_UAV,:) 0];
             discon(1,idx_UAV) = 1; % agent 3 is not connected 
             if idx_UAV ~= 3         % agent 3 is not connected
                SetTaskAllocation(1) = 1 ; 
             end
        end
        
    end
  

%     SetTaskAllocation(1)=1;
    STA =SetTaskAllocation(1) ;
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
    end
 
    
    for idx_UAV = 1 : NUM_UAV
        if( discon(1,idx_UAV) == 1 )
             MinDistTask(idx_UAV)= inf  ;
        end
    end
    
    % Task Re-assignment
    if (SetTaskAllocation(1))        
        for iter_UAVx = 1 : NUM_UAV-1
            for iter_UAVy = iter_UAVx+1 : NUM_UAV
                if (discon(1,iter_UAVy) ~= 1) 
                if ((MinDistTask(iter_UAVx) ~= inf) || (MinDistTask(iter_UAVy) ~= inf))
                    if (idxMinTask(iter_UAVx) == idxMinTask(iter_UAVy))
                        if (MinDistTask(iter_UAVx) < MinDistTask(iter_UAVy))
                            TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.
                            MinDistTask(iter_UAVy) = inf;
                             [wpt_temp, second_choice_temp, temp] = ...
                            operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVx)-1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
                            vrt_config,vrt_config,...
                            HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_task(idxMinTask(iter_UAVx),:),...
                            iter_wpt(iter_UAVx)-1);
                            wpt(1:length(wpt_temp(:,1)),(iter_UAVx-1)*4+1:(iter_UAVx*4)) = wpt_temp;
                            second_choice(1,(iter_UAVx-1)*4+1:iter_UAVx*4) = second_choice_temp;
% %                             TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.
                             clear wpt_temp second_choice_temp;
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
                             [wpt_temp, second_choice_temp, temp] = ...
                            operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
                            vrt_config,vrt_config,...
                            HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_task(idxMinTask(iter_UAVy),:),...
                            iter_wpt(iter_UAVy)-1);
                            wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
                            second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
% %                             TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.
                             clear wpt_temp second_choice_temp;
                            
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
        end
        
    for idx_UAV = 1 : NUM_UAV
        if( discon(1,idx_UAV) == 1 )
                   [wpt_temp, second_choice_temp, temp] = ...
                        operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(idx_UAV)-1,(idx_UAV)*4+1:idx_UAV*4),...
                        vrt_config,vrt_config,...
                        HLI_pos(iter_prcd,(idx_UAV-1)*3+1:(idx_UAV-1)*3+2),p_f(idx_UAV,:),...
                        iter_wpt(idx_UAV)-1);
                    wpt(1:length(wpt_temp(:,1)),(idx_UAV-1)*4+1:(idx_UAV*4)) = wpt_temp;
                    second_choice(1,(idx_UAV-1)*4+1:idx_UAV*4) = second_choice_temp;
                    clear wpt_temp second_choice_temp;
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
                HLI_pos(iter_prcd,(SetTaskAllocation(2)-1)*3+1:(SetTaskAllocation(2)-1)*3+2),p_f((SetTaskAllocation(2)),:),...
                iter_wpt(iter_UAV)-1);
            wpt(1:length(wpt_temp(:,1)),(SetTaskAllocation(2)-1)*4+1:(SetTaskAllocation(2)*4)) = wpt_temp;
            second_choice(1,(SetTaskAllocation(2)-1)*4+1:SetTaskAllocation(2)*4) = second_choice_temp;
            clear wpt_temp second_choice_temp;
        end
       
        TASK_STATUS_prev = TASK_STATUS ; 
    end
    

    % calculate the UAV position
%     for iter_UAV = 1 : NUM_UAV
%         if (cnt_terminate(iter_UAV) == 0)
%             [HLI_pos(iter_prcd+1,(iter_UAV-1)*3+1:iter_UAV*3),flag_change] = ...
%                 calculate_postion(HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3),wpt(iter_wpt(iter_UAV),(iter_UAV-1)*4+1:(iter_UAV-1)*4+2),HLI_vel(iter_UAV),HLI_max_dyaw(iter_UAV),dt,chk_range);
%             if (flag_change == 1)
%                 cnt(iter_UAV) = cnt(iter_UAV) - 1;
%                 if cnt(iter_UAV) == 0
%                     iter_wpt(iter_UAV) = iter_wpt(iter_UAV) + 1;
%                     cnt(iter_UAV) = chk_range/dt+3;
%                 end
%             end
%         else
%             HLI_pos(iter_prcd+1,(iter_UAV-1)*3+1:iter_UAV*3) = HLI_pos(iter_prcd,(iter_UAV-1)*3+1:iter_UAV*3);
%         end
%     end
    
    
    SetTaskAllocation(1) = 0;  % event trigger!


    
    % check goal point. if so, this procedure should be terminated.
    % otherwise, procedure is proceeded.
    for idx_UAV = 1 : NUM_UAV
        
        if distance(HLI_pos(iter_prcd,(idx_UAV-1)*3+1:(idx_UAV-1)*3+2),p_f(idx_UAV,1:2)) < chk_range
            cnt_terminate(idx_UAV) = 1;
        end
        
        %%% Do the TASK
        if distance(HLI_pos(iter_prcd,(idx_UAV-1)*3+1:(idx_UAV-1)*3+2),p_task(idxMinTask(idx_UAV),:)) < chk_range
            SetTaskAllocation = [1,idx_UAV];  % event trigger!
            TASK_STATUS(1,idxMinTask(idx_UAV)) = 3; % Task is done
            iter_wpt = 2*ones(1,NUM_UAV);                     
        end
        

        if distance(HLI_pos(iter_prcd,(idx_UAV-1)*3+1:(idx_UAV-1)*3+2),wpt(2,(idx_UAV-1)*4+1:(idx_UAV-1)*4+4)) < chk_range           
            SetTaskAllocation = [1,idx_UAV];  % event trigger!
        end
        
    end
    

 % plotting the wpt (position)
            if(mod(cnt,10)==0  ) 
                plot(HLI_pos(iter_prcd,3*(0)+1),HLI_pos(iter_prcd,3*(0)+2),'r*');hold on ; 
                plot(HLI_pos(iter_prcd,3*(1)+1),HLI_pos(iter_prcd,3*(1)+2),'g*');hold on ; 
                plot(HLI_pos(iter_prcd,3*(2)+1),HLI_pos(iter_prcd,3*(2)+2),'b*');hold on ; 
%                 plot(wpt(1,5),wpt(1,6),'ro');
                grid on; 
                axis equal;
            end
            
            if(mod(cnt,10)==0)
                cntt = cntt + 1 ;
                POS(cntt,1:2) = [HLI_pos(iter_prcd,3*(0)+1) HLI_pos(iter_prcd,3*(0)+2)];
                POS(cntt,3:4) = [HLI_pos(iter_prcd,3*(1)+1) HLI_pos(iter_prcd,3*(1)+2)];
                POS(cntt,5:6) = [HLI_pos(iter_prcd,3*(2)+1) HLI_pos(iter_prcd,3*(2)+2)];
                TASK_list(cntt, 1:NUM_TASK) = TASK_STATUS(1,:);
                TASK_list(cntt, NUM_TASK+1:2*NUM_TASK) = TASK_STATUS(2,:);
            end
                
         cnt = cnt+1 ;

    if STA == 0 
       TASK_STATUS = TASK_STATUS_prev ;  
    else
        TASK_STATUS
           for iter = 1 : NUM_TASK
                if (TASK_STATUS(1,iter) ~= 3)
                    TASK_STATUS(:,iter) = 0; % Task is rebooted
                end
           end
    end

    out = [wpt(2,1:NUM_UAV*4)] ;
    
end
