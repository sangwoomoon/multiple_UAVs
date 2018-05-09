function [flag] = calculate_obstacle_avoid(OBS_VRT,vrt_config,wpt)

% increasing
dist_1 = distance(vrt_config((OBS_VRT+1)*wpt(1,3)+wpt(1,4),[1,2]),vrt_config((OBS_VRT+1)*wpt(1,3)+wpt(1,4)+1,[1,2]));

% decreasing
if (wpt(1,4) == 1)
    dist_2 = distance(vrt_config((OBS_VRT+1)*wpt(1,3)+wpt(1,4),[1,2]),vrt_config((OBS_VRT+1)*wpt(1,3)+OBS_VRT,[1,2]));
else
    dist_2 = distance(vrt_config((OBS_VRT+1)*wpt(1,3)+wpt(1,4),[1,2]),vrt_config((OBS_VRT+1)*wpt(1,3)+wpt(1,4)-1,[1,2]));
end

if (dist_1 > dist_2)
    flag = 1;
else
    flag = 0;
end
