function [OBJ_new,OBJ_Ysec,OBJ_Yslp] = calculate_config_space(OBJ,OBJ_SIZE,OBJ_cw_ccw,RANGE)

OBJ_Ysec = zeros(1,length(OBJ));
OBJ_Yslp = zeros(1,length(OBJ));
for i = 1 : OBJ_SIZE
    if abs(OBJ(i+1,1) - OBJ(i,1)) < 1e-3
        OBJ(i+1,1) = OBJ(i+1,1) + 1e-3;
    end
    OBJ_Yslp(1,i) = (OBJ(i+1,2)-OBJ(i,2))/(OBJ(i+1,1)-OBJ(i,1));
    OBJ_Ysec(1,i) = -OBJ_Yslp(1,i)*OBJ(i,1)+OBJ(i,2);
    if (OBJ_cw_ccw == 0)
        OBJ_Ysec(1,i) = OBJ_Ysec(1,i) - RANGE/(cos(atan2(OBJ(i+1,2)-OBJ(i,2),(OBJ(i+1,1)-OBJ(i,1)))));
    elseif (OBJ_cw_ccw == 1)
        OBJ_Ysec(1,i) = OBJ_Ysec(1,i) + RANGE/(cos(atan2(OBJ(i+1,2)-OBJ(i,2),(OBJ(i+1,1)-OBJ(i,1)))));       
    end        
end

OBJ_Yslp(1,length(OBJ)) = OBJ_Yslp(1,1);
OBJ_Ysec(1,length(OBJ)) = OBJ_Ysec(1,1);

OBJ_new = ones(1,2);

for i = 1 : OBJ_SIZE
    OBJ_new(i+1,1) = (OBJ_Ysec(1,i)-OBJ_Ysec(1,i+1))/(-OBJ_Yslp(1,i)+OBJ_Yslp(1,i+1));
    OBJ_new(i+1,2) = (OBJ_Yslp(1,i+1)*OBJ_Ysec(1,i)-OBJ_Yslp(1,i)*OBJ_Ysec(1,i+1))/(-OBJ_Yslp(1,i)+OBJ_Yslp(1,i+1));
end

OBJ_new(1,:) = OBJ_new(OBJ_SIZE+1,:);