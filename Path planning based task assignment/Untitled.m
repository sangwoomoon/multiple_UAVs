simulation_TA(Pos_XY_UAV(1,:),Pos_XY_UAV(2,:))


Pos_XY_UAV(1,:) =[ -145  285  ];
Pos_XY_UAV(2,:) =[ -140 300  ];

 -200.0000   25.0000  -21.6739   30.5439
 -233.4144  110.9994   30.3295   86.5983
 
 HLI_pos = [-20 50 0 20 170 0 ]
 Pos_XY_UAV(1,:) = [-20 50] ; 
 Pos_XY_UAV(2,:) = [20 170] ;
 
 HLI_pos=[ -145  285 0 -140 300 0 ];
 
 iter_UAV =1; 
 u1 = [iter_UAV,Pos_XY_UAV(iter_UAV,:),wpt(1,1:8),wpt(2,1:8),TASK_STATUS1,TASK_STATUS2,[1 0]] ;
 ans1 = simulation_TA_ini(u1)
 idxMinTask(1,1) = ans1(1,2)
 MinDistTask(1,1) = ans1(1,3)
 HLI_posi(1,1:2) = ans1(1,4:5)
 
 iter_UAV =2; 
 u2 = [iter_UAV,Pos_XY_UAV(iter_UAV,:),wpt(1,1:8),wpt(2,1:8),TASK_STATUS1, TASK_STATUS2,[1 0]] ;
 ans2 = simulation_TA_ini(u2)
 idxMinTask(1,2) = ans2(1,2)
 MinDistTask(1,2) = ans2(1,3)
 HLI_posi(1,1:2) = ans2(1,4:5)
 
  u = [iter_UAV,HLI_posi,TASK_STATUS1,TASK_STATUS2,[1 0],wpt(1,:),wpt(2,:),idxMinTask,MinDistTask] ;
 simulation_TA_ReTA(u)
 
 
%  u = [wpt(1,(iter_UAV-1)*4+1:(iter_UAV-1)*4+4),wpt(2,(iter_UAV-1)*4+1:(iter_UAV-1)*4+4),TASK_STATUSi,SetTaskAllocation];
 
 u = ans ; 
 simulation_TA_center3(u)