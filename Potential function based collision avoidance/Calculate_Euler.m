function [euler] = Calculate_Euler(angle)

roll = angle(1);
pitch = angle(2);
yaw = angle(3);

euler = [cos(yaw)*cos(pitch) cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw) cos(roll)*cos(yaw)*sin(pitch)+sin(roll)*sin(yaw);
    sin(yaw)*cos(pitch) sin(yaw)*sin(roll)*sin(pitch)+cos(roll)*cos(yaw) cos(roll)*sin(yaw)*sin(pitch)-sin(roll)*cos(yaw);
    -sin(pitch) cos(pitch)*sin(roll) cos(roll)*cos(pitch)];