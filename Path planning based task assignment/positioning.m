function out = positioning(Pos_XY,Pos_XY_Ref)


vel = 1 ; 
dt = 0.4 ; 

theta = atan2(Pos_XY_Ref(2)-Pos_XY(2),Pos_XY_Ref(1)-Pos_XY(1)); 


Pos_XY(1) = Pos_XY+vel*dt*cos(theta) ;
Pos_XY(2) = Pos_XY+vel*dt*sin(theta) ;


if distance(Pos_XY,Pos_XY_Ref) < chk_range
    flag_change = 1;
else
    flag_change = 0;
end

out = [Pos_XY, flag_change] ; 

end