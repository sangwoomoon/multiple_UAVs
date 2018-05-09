function [chk_mpt] = check_meeting_points(OBS_NUM,OBS_VRT,v,wpt_1,wpt_2)

chk_mpt = 0;
for idx_bldg = 1 : OBS_NUM
    for idx_edge = 1 : OBS_VRT
        a = (v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,2)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2))/(v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1));   
        b = (wpt_1(1,2)-wpt_2(1,2))/(wpt_1(1,1)-wpt_2(1,1));

        mpt_tmp(1,1) = (-a*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+b*wpt_2(1,1)-wpt_2(1,2))/(-a+b);
        mpt_tmp(1,2) = (-a*b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+a*b*wpt_2(1,1)-a*wpt_2(1,2))/(-a+b);
        
        if (determine_on_line(mpt_tmp,v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,[1,2]),v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,[1,2])))   
            if (determine_on_line(mpt_tmp,wpt_1,wpt_2))
                if ((abs(mpt_tmp(1,1)-wpt_1(1,1)) >= 1) && (abs(mpt_tmp(1,2)-wpt_1(1,2)) >= 1)) && ((abs(mpt_tmp(1,1)-wpt_2(1,1)) >= 1) && (abs(mpt_tmp(1,2)-wpt_2(1,2)) >= 1))
                    chk_mpt = 1;
                end
            end
        end
    end
end
