function [meet_pt,det_mpt] = find_obstacles(KNOBS_NUM,OBS_NUM,OBS_VRT,v,wpt_1,wpt_2,flag_scan)

% meet_pts type : [x y idx_bldg idx_edge t(parameter, from lower edge # to upper edge #)]

% to identify scanned obstacles, KNOBS_NUM is added to the index of
% building.

det_mpt = 0;
index = 1;
for idx_bldg = 1 : OBS_NUM
    for idx_edge = 1 : OBS_VRT
        %%%singular case를 막기 위함.
        if abs(v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)) < 0.0000001
            v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1) = v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1) + 0.0001;
        end
        if abs((wpt_1(1,1)-wpt_2(1,1))) < 0.0000001
            wpt_1(1,1) = wpt_1(1,1) + 0.0001;
        end
        
        a = (v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,2)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2))/(v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1));   
        b = (wpt_1(1,2)-wpt_2(1,2))/(wpt_1(1,1)-wpt_2(1,1));

        mpt_tmp(1,1) = (-a*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+b*wpt_2(1,1)-wpt_2(1,2))/(-a+b);
        mpt_tmp(1,2) = (-a*b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+a*b*wpt_2(1,1)-a*wpt_2(1,2))/(-a+b);
        
        if (determine_on_line(mpt_tmp,v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,[1,2]),v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,[1,2])))   
            if (determine_on_line(mpt_tmp,wpt_1,wpt_2))
                t = norm(mpt_tmp-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1:2))/norm(v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1:2)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1:2));
                meet_pt(index,:) = [mpt_tmp idx_bldg+KNOBS_NUM idx_edge t];
                det_mpt = 1;
                index = index + 1;
            end
        end
    end
end

if (det_mpt == 1)
    if (length(meet_pt(:,1)) >= 2) && (flag_scan ~= 5)
        min = distance(meet_pt(1,1:2),wpt_1);
        meet_pt_res = meet_pt(1,:);
        for i = 2 : length(meet_pt(:,1))
            if (min > distance(meet_pt(i,1:2),wpt_1))
                min = distance(meet_pt(i,1:2),wpt_1);
                meet_pt_res = meet_pt(i,:);
            end
        end
        clear meet_pt;
        meet_pt = meet_pt_res;
    end
end

if (det_mpt == 0)
    meet_pt = 0;
end