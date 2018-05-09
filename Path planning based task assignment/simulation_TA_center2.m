function out = simulation_TA_center2(u)

    global NUM_UAV
    
    
    for iter = 1 : NUM_UAV
        
    idxMinTask(1,iter) = u((iter-1)*2+1) ;
    MinDistTask(1,iter) = u((iter-1)*2+2) ; 
        
    end
%     
%     idxMinTask(1,1) = u(2) ;
%     MinDistTask(1,1) = u(3) ; 
%     idxMinTask(1,2) = u(21) ;
%     MinDistTask(1,2) = u(22) ;     
    
    out = [idxMinTask MinDistTask];
end 