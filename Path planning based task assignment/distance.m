function [dist] = distance (a,b)
    dist = sqrt((a(1,1)-b(1,1))^2+(a(1,2)-b(1,2))^2);