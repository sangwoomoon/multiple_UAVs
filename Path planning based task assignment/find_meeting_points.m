function [meet_pts,det_meet_pt] = find_meeting_points (OBS_NUM,OBS_VRT,v,final_pt,wpt_prev)

% Find the meeting points.
%  find the meeting points by using the fundamental equation for line-line
%  meeting. the meeting points on the line which is from the previous
%  waypoint (or starting point) to the final point (or goal point).
%
%%%% REVISION NOTE %%%
%
% REVISION VERSION : 1.10
% REVISION DATE : 2009. 11. 2
% REVISED BY SANGWOO MOON.

index = 1;
det_meet_pt = 0;
for idx_bldg = 1 : OBS_NUM
    for idx_edge = 1 : OBS_VRT
        a = (v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,2)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2))/(v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1));   
        b = (final_pt(1,2)-wpt_prev(1,2))/(final_pt(1,1)-wpt_prev(1,1));

        mpt_tmp(1,1) = (-a*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+b*wpt_prev(1,1)-wpt_prev(1,2))/(-a+b);
        mpt_tmp(1,2) = (-a*b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+a*b*wpt_prev(1,1)-a*wpt_prev(1,2))/(-a+b);

        if (determine_on_line(mpt_tmp,v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,[1,2]),v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,[1,2])))
            if (determine_on_line(mpt_tmp,wpt_prev(1,[1,2]),final_pt))
                if (abs(final_pt(1,1)-mpt_tmp(1,1)) >= 0.01) || (abs(final_pt(1,2)-mpt_tmp(1,2))>= 0.01)
                    meet_pts(index,1) = mpt_tmp(1,1);
                    meet_pts(index,2) = mpt_tmp(1,2);
                    meet_pts(index,[3,4]) = v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,[3,4]);
                    index = index + 1;
                    det_meet_pt = 1;
                end
            end
        end
    end
end

% clutter case.
if (length(v(:,1)) > OBS_NUM*(OBS_VRT+1))
    for idx_line = OBS_NUM*(OBS_VRT+1)+1 : 2 : length(v(:,1))-1
        a = (v(idx_line+1,2)-v(idx_line,2))/(v(idx_line+1,1)-v(idx_line,1));   
        b = (final_pt(1,2)-wpt_prev(1,2))/(final_pt(1,1)-wpt_prev(1,1));

        mpt_tmp(1,1) = (-a*v(idx_line,1)+v(idx_line,2)+b*wpt_prev(1,1)-wpt_prev(1,2))/(-a+b);
        mpt_tmp(1,2) = (-a*b*v(idx_line,1)+b*v(idx_line,2)+a*b*wpt_prev(1,1)-a*wpt_prev(1,2))/(-a+b);

        if (determine_on_line(mpt_tmp,v(idx_line,[1,2]),v(idx_line+1,[1,2])))
            if (determine_on_line(mpt_tmp,wpt_prev(1,[1,2]),final_pt))
                if (abs(final_pt(1,1)-mpt_tmp(1,1)) >= 0.01) || (abs(final_pt(1,2)-mpt_tmp(1,2))>= 0.01)
                    meet_pts(index,1) = mpt_tmp(1,1);
                    meet_pts(index,2) = mpt_tmp(1,2);
                    meet_pts(index,[3,4]) = v(idx_line,[3,4]);
                    index = index + 1;
                    det_meet_pt = 1;
                end
            end
        end
    end
end

if det_meet_pt == 0
    meet_pts = zeros(1,4);
end