function out = MILP(u)

global wpt_list cnt wpt NUM_UAV p_i chk_range NUM_TASK cntt POSI cnttt 
global popup

clock_zero = u(NUM_UAV*2+1) ; 

for idx_UAV = 1 : NUM_UAV
   pos(1,2*(idx_UAV-1)+1) = u((idx_UAV-1)*2+1) ;
   pos(1,2*(idx_UAV-1)+2) = u((idx_UAV-1)*2+2) ;
end


  if (clock_zero == 0 )
        for iter_UAV = 1 : NUM_UAV
            pos(1,(iter_UAV-1)*2+1:iter_UAV*2) = [p_i(iter_UAV,:)];
        end
    end

if popup == 1 
   cnt = ones(1,NUM_UAV) ; 
   popup = 2 ; 
end
    
for iduav = 1 : NUM_UAV

   
    if distance(pos(1,2*(iduav-1)+1:2*iduav),wpt(1,2*(iduav-1)+1:2*iduav)) < chk_range
        if (cnt(1,iduav) < (NUM_TASK+1))
        cnt(1,iduav)=[ cnt(1,iduav)+1 ];   
        wpt(1,(iduav-1)*2+1:iduav*2) = wpt_list(cnt(1,iduav),(iduav-1)*2+1:iduav*2) ;
        end
    end
    
end
            if(mod(cntt,10)==0  ) 
                plot(pos(1,2*(0)+1),pos(1,2*(0)+2),'r*');hold on ; 
                plot(pos(1,2*(1)+1),pos(1,2*(1)+2),'g*');hold on ; 
                plot(pos(1,2*(2)+1),pos(1,2*(2)+2),'b*');hold on ; 
%                 plot(wpt(1,5),wpt(1,6),'ro');
                grid on; 
                axis equal;
                
                cnttt = cnttt +1 ; 
                POSI(cnttt,1:2)=[pos(1,2*(0)+1),pos(1,2*(0)+2)] ;
                POSI(cnttt,3:4)=[pos(1,2*(1)+1),pos(1,2*(1)+2)] ;
                POSI(cnttt,5:6)=[pos(1,2*(2)+1),pos(1,2*(2)+2)] ;
                
                
            end
            
            cntt = cntt +1 ; 



 out = wpt  ;

end