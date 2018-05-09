function [det] = check_fibonacci_filtering(wpt_tmp,wpt_prev,wpt,OBS_NUM,OBS_VRT,vrt_config)

% Check Fibonacci filtering
%  There is the three neighboring waypoints that the middle waypoint can be
%  cancelled out because the path which connects between the other waypoints
%  are admissible. By the optimal principle, there is better path which
%  cancels the middle waypoint. Fibonacci filtering, which is named from
%  Fibonacci series, checks this situation and cancels that waypoint. 
%
%%%% REVISION NOTE %%%
%
% REVISION VERSION : 1.10b
% 1ST REVISION DATE : 2009. 11. 3
% 2ND REVISION DATE : 2009. 11. 5 
% REVISED BY SANGWOO MOON.


% Find the meeting point :
%  use the "find_meeting_points.m" with new waypoint and previous one.
[meet_pts,det_meet_pts] = find_meeting_points(OBS_NUM,OBS_VRT,vrt_config,wpt_tmp,wpt_prev);
meet_pts(:,5) = 0;

if (det_meet_pts == 0)
    det = 0;
else
    det = 1;
end

% Among meeting points, there should be previous waypoints becuase of
% calculation. To prevent, erase meeting points which is the same as 
% the previous waypoints.
for iter_mtpt = 1 : length(meet_pts(:,1))
    meet_pts(iter_mtpt,5) = 1;
    for iter_wpt = 1 : length(wpt(:,1))
        if abs(meet_pts(iter_mtpt,1)-wpt(iter_wpt,1)) < 0.001 && abs(meet_pts(iter_mtpt,2)-wpt(iter_wpt,2)) < 0.001 % in mathematical represent, meet_pts and wpt are same.
            meet_pts(iter_mtpt,5) = 0;
        end
    end
end

% if the FRESH point is detected, we can't use this filtering.
% Check this one.
if max(meet_pts(:,5)) == 0
    det = 0;
end

% However, this filtering shouldn't operate if those waypoints are on the
% same obstacles. Check this one. (USE INPOLYGON, but this function should
% be modifieded.
if (wpt_tmp(1,3) == wpt_prev(1,3)) && inpolygon((wpt_tmp(1,1)+wpt_prev(1,1))/2,(wpt_tmp(1,2)+wpt_prev(1,2))/2,vrt_config((wpt_tmp(1,3)-1)*(OBS_VRT+1)+1:(wpt_tmp(1,3))*(OBS_VRT+1),1),vrt_config((wpt_tmp(1,3)-1)*(OBS_VRT+1)+1:(wpt_tmp(1,3))*(OBS_VRT+1),2))
    det = 1;
end

    
