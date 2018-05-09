function out = simulation_TA_center3(u) 

    global NUM_UAV NUM_TASK TASK_STATUS cnt

  if isempty(cnt)
      cnt =1 ; 
  end 
  
  num_data = 2*4+NUM_TASK*2+2 ; 
  
  for iter = 1 : NUM_UAV 
      
    wpt(1,(iter-1)*4+1:(iter-1)*4+4) = u((iter-1)*num_data+1:(iter-1)*num_data+4) ;
    wpt(2,(iter-1)*4+1:(iter-1)*4+4) = u((iter-1)*num_data+5:(iter-1)*num_data+8) ;
    
   TASK_STATUSi((iter-1)*2+1,1:NUM_TASK) = [ u((iter-1)*num_data+9:(iter-1)*num_data+14) ] ; 
   TASK_STATUSi((iter-1)*2+2,1:NUM_TASK) = [ u((iter-1)*num_data+15:(iter-1)*num_data+20) ] ; 
   
    for iter_k = 1 : NUM_TASK
      if  ( (TASK_STATUSi((iter-1)*2+1,iter_k) == 3) && ( TASK_STATUSi((iter-1)*2+2,iter_k) == iter ) )
          TASK_STATUS(1:2,iter_k) = [ 3 ; iter]; % done 
      elseif ( (TASK_STATUSi((iter-1)*2+1,iter_k) == 1) &&  (TASK_STATUSi((iter-1)*2+2,iter_k) == iter) && (TASK_STATUS(1,iter_k) ~= 3 ))
          TASK_STATUS(1:2,iter_k) = [ 1 ; iter]; % doing
      elseif ( (TASK_STATUSi((iter-1)*2+1,iter_k) == 2 )&&  (TASK_STATUSi((iter-1)*2+2,iter_k) == iter )&& (TASK_STATUS(1,iter_k) ~= 3 ))
          TASK_STATUS(1:2,iter_k) = [ 2 ; iter]; % no available (other agent doing) 
      end           
    end

  end
   
  
   % Summation of SetTaskAllocation signals
   if (u(num_data*NUM_UAV+1) == 0 ) 
        SetTaskAllocation = [0 0] ;
   else
        SetTaskAllocation = [1 0] ; 
   end
   

  TASK_STATUS  

    out = [wpt(1,:) wpt(2,:) TASK_STATUS(1,:) TASK_STATUS(2,:) SetTaskAllocation] ;
  
    % Reset the TASK_STATUS excepting status 3 (done) 
            for iter = 1 : NUM_TASK
                if (TASK_STATUS(1,iter) ~= 3)
                    TASK_STATUS(:,iter) = zeros(2,1) ;  
                end             
            end

   
    % plotting the wpt (position)
    for iter = 1 : NUM_UAV
            if(mod(cnt,150)==0  ) 
                plot(wpt(1,(iter-1)*4+1),wpt(1,(iter-1)*4+2),'r*');
%                 plot(wpt(1,5),wpt(1,6),'ro');
                grid on;
                axis equal;
            end
    end
            
          cnt = cnt+1;

end