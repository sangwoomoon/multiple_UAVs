function out = simulation_TA_ReTA(u)

    global NUM_UAV NUM_TASK chk_range
    global OBS_NUM OBS_VRT vrt_config  p_f p_task 
 
    
    iter_UAV(1,1) = u(1) ;
    HLI_posi(1,1:2) = u(2:3) ; 
    TASK_STATUS(1,1:6) = u(4:9) ; 
    TASK_STATUS(2,1:6) = u(10:15) ; 
    SetTaskAllocation(1,1:2) = u(16:17) ; 
    wpt(1,1:8) = u(18:25) ; 
    wpt(2,1:8) = u(26:33) ;  
    idxMinTask(1,1:2) = u(34:35) ;
    MinDistTask(1,1:2) = u(36:37) ; 

    iter_wpt = 2*ones(1,NUM_UAV);
    iter_prcd = 1 ; 
    HLI_pos(iter_prcd,(iter_UAV-1)*3+1:(iter_UAV-1)*3+2) = HLI_posi ; 
    
   % Task Re-assignment
    if (SetTaskAllocation(1))        
%         for iter_UAVx = 1 : NUM_UAV-1
            iter_UAVx = iter_UAV ; 
            for iter_UAVy = 1 : NUM_UAV
                if (iter_UAVx ~= iter_UAVy) 
                if ((MinDistTask(iter_UAVx) ~= inf) || (MinDistTask(iter_UAVy) ~= inf))
                    if (idxMinTask(iter_UAVx) == idxMinTask(iter_UAVy))
                        if (MinDistTask(iter_UAVx) < MinDistTask(iter_UAVy))
                            TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.
                            MinDistTask(iter_UAVy) = inf;
                            [wpt_temp, second_choice_temp, temp] = ...
                        operate_algorithm(OBS_NUM,OBS_VRT,wpt(1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
                        vrt_config,vrt_config,...
                        HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_task(idxMinTask(iter_UAVx),:),...
                        1);
                        wpt(1:length(wpt_temp(:,1)),(iter_UAVx-1)*4+1:(iter_UAVx*4)) = wpt_temp;
                        second_choice(1,(iter_UAVx-1)*4+1:iter_UAVx*4) = second_choice_temp;
                   
%                             for iter_TASK = 1 : NUM_TASK
%                                 if (DistPathForTask(iter_UAVy,iter_TASK) < MinDistTask(iter_UAVy)) && (TASK_STATUS(1,iter_TASK) == 0)
%                                     MinDistTask(iter_UAVy) = DistPathForTask(iter_UAVy,iter_TASK);
%                                     idxMinTask(iter_UAVy) = iter_TASK;
%                                 end
%                             end
%                             if MinDistTask(iter_UAVy) == inf
%                                 [wpt_temp, second_choice_temp, temp] = ...
%                                     operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
%                                     vrt_config,vrt_config,...
%                                     HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_f(iter_UAVy,:),...
%                                     iter_wpt(iter_UAVy)-1);
%                                 wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
%                                 second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
%                             else
%                                 [wpt_temp, second_choice_temp, temp] = ...
%                                     operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
%                                     vrt_config,vrt_config,...
%                                     HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_task(idxMinTask(iter_UAVy),:),...
%                                     iter_wpt(iter_UAVy)-1);
%                                 wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
%                                 second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
%                                 TASK_STATUS(:,idxMinTask(iter_UAVy)) = [1;iter_UAVy]; % TASK status update.
%                             end
%                         elseif ( iter_UAVx < iter_UAVy ) 
%                              TASK_STATUS(:,idxMinTask(iter_UAVx)) = [1;iter_UAVx]; % TASK status update.
%                             MinDistTask(iter_UAVy) = inf;
%                              [wpt_temp, second_choice_temp, temp] = ...
%                         operate_algorithm(OBS_NUM,OBS_VRT,wpt(1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
%                         vrt_config,vrt_config,...
%                         HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_task(idxMinTask(iter_UAVx),:),...
%                         1);
%                         wpt(1:length(wpt_temp(:,1)),(iter_UAVx-1)*4+1:(iter_UAVx*4)) = wpt_temp;
%                         second_choice(1,(iter_UAVx-1)*4+1:iter_UAVx*4) = second_choice_temp;
                        else
                            TASK_STATUS(:,idxMinTask(iter_UAVy)) = [1;iter_UAVy]; % TASK status update.
                            MinDistTask(iter_UAVx) = inf;
                 for iter_TASK = 1 : NUM_TASK
                    if (TASK_STATUS(1,iter_TASK) ~= 3)
                    [wpt_temp, second_choice_temp, DistPathForTask(iter_UAVx,iter_TASK)] = ...
                        operate_algorithm(OBS_NUM,OBS_VRT,wpt(1,(iter_UAVx-1)*4+1:iter_UAVx*4),...
                        vrt_config,vrt_config,...
                        HLI_pos(iter_prcd,(iter_UAVx-1)*3+1:(iter_UAVx-1)*3+2),p_task(iter_TASK,:),...
                        1);
                      if (length(wpt_temp(:,1)) == 2)
                        DistPathForTask(iter_UAVx,iter_TASK) = norm(wpt_temp(2,1:2) - wpt_temp(1,1:2));
                      end
                      DistPathForTask(iter_UAVx,iter_TASK) = DistPathForTask(iter_UAVx,iter_TASK); % + norm(wpt_temp(length(wpt_temp(:,1)),1:2)-p_f(iter_UAV,1:2));
                    else
                      DistPathForTask(iter_UAVx,iter_TASK) = inf;
                    end
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
% 
%                         [wpt_temp, second_choice_temp, temp] = ...
%                             operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
%                             vrt_config,vrt_config,...
%                             HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_task(idxMinTask(iter_UAVy),:),...
%                             iter_wpt(iter_UAVy)-1);
%                         wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
%                         second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
%                         clear wpt_temp second_choice_temp;
%                         TASK_STATUS(:,idxMinTask(iter_UAVy)) = [1;iter_UAVy]; % TASK status update.
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

%                     [wpt_temp, second_choice_temp, temp] = ...
%                         operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(iter_UAVy)-1,(iter_UAVy-1)*4+1:iter_UAVy*4),...
%                         vrt_config,vrt_config,...
%                         HLI_pos(iter_prcd,(iter_UAVy-1)*3+1:(iter_UAVy-1)*3+2),p_f(iter_UAVy,:),...
%                         iter_wpt(iter_UAVy)-1);
%                     wpt(1:length(wpt_temp(:,1)),(iter_UAVy-1)*4+1:(iter_UAVy*4)) = wpt_temp;
%                     second_choice(1,(iter_UAVy-1)*4+1:iter_UAVy*4) = second_choice_temp;
%                     clear wpt_temp second_choice_temp;
                end
               end
            end
%         end
%         chk_FINAL = 0;
%         for iter_FINAL = 1 : NUM_UAV
%             if (MinDistTask(iter_FINAL) ~= inf)
%                 chk_FINAL = chk_FINAL + 1;
%             end
%         end
%         if chk_FINAL == 0 % final state (the last UAV should go to the goal point)
%             [wpt_temp, second_choice_temp, temp] = ...
%                 operate_algorithm(OBS_NUM,OBS_VRT,wpt(iter_wpt(SetTaskAllocation(2))-1,(SetTaskAllocation(2)-1)*4+1:SetTaskAllocation(2)*4),...
%                 vrt_config,vrt_config,...
%                 HLI_pos(iter_prcd,(SetTaskAllocation(2)-1)*3+1:(SetTaskAllocation(2)-1)*3+2),p_f(idxMinTask(SetTaskAllocation(2)),:),...
%                 iter_wpt(iter_UAV)-1);
%             wpt(1:length(wpt_temp(:,1)),(SetTaskAllocation(2)-1)*4+1:(SetTaskAllocation(2)*4)) = wpt_temp;
%             second_choice(1,(SetTaskAllocation(2)-1)*4+1:SetTaskAllocation(2)*4) = second_choice_temp;
%             clear wpt_temp second_choice_temp;
%        end
        
        SetTaskAllocation(1) = 0 ;  
        
%         if distance(HLI_pos(iter_prcd+1,(iter_UAV-1)*3+1:(iter_UAV-1)*3+2),p_f(iter_UAV,1:2)) < chk_range
%             cnt_terminate(idx_UAV) = 1;
%         end
        %%% Do the TASK
        if distance(HLI_pos(iter_prcd,(iter_UAV-1)*3+1:(iter_UAV-1)*3+2),p_task(idxMinTask(iter_UAV),:)) < chk_range
            SetTaskAllocation = [1,iter_UAV];  % event trigger!
            TASK_STATUS(1,idxMinTask(iter_UAV)) = 3; % Task is done
            iter_wpt = 2*ones(1,NUM_UAV);
            for iter = 1 : NUM_TASK
                if (TASK_STATUS(1,iter) ~= 3)
                    TASK_STATUS(:,iter) = 0; % Task is rebooted
                end
            end
        end
   
    end
    
    TASK_STATUSi = zeros(1,2) ; 
    for iter = 1 : NUM_TASK
        if (TASK_STATUS(2,iter) == iter_UAV ) 
        TASK_STATUSi = [TASK_STATUS(1,iter) iter];
        end
    end
    
    wpt_xplane = wpt ; 
%      out = [wpt] ;%(2,(iter_UAV-1)*4+1:(iter_UAV-1)*4+2)
    out = [wpt(1,(iter_UAV-1)*4+1:(iter_UAV-1)*4+4),wpt(2,(iter_UAV-1)*4+1:(iter_UAV-1)*4+4),TASK_STATUSi,SetTaskAllocation] ;
end 