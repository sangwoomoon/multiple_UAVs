function [OBJ_cw_ccw] = determine_cw_ccw(OBJ)

OBJ_angle = atan2(OBJ(2,2)-OBJ(1,2),OBJ(2,1)-OBJ(1,1));
OBJ_deter = [OBJ(3,1)-OBJ(1,1),OBJ(3,2)-OBJ(1,2)];
OBJ_det_cw = OBJ_deter(1,1)*sin(-OBJ_angle)+OBJ_deter(1,2)*cos(-OBJ_angle);
if (OBJ_det_cw <= 0)
    OBJ_cw_ccw = 1; % CW
else
    OBJ_cw_ccw = 0; % CCW
end
end