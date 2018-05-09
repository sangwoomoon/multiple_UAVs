function [vrt] = invite_marking(OBS_VRT,vrt,wpt,idx_wpt)

% vertex marking
vrt((OBS_VRT+1)*(wpt(idx_wpt,3)-1)+wpt(idx_wpt,4),6) = 1;
if wpt(idx_wpt,4) == 1
    vrt((OBS_VRT+1)*(wpt(idx_wpt,3)-1)+OBS_VRT+1,6) = 1;
end

% bilding marking
if (wpt(idx_wpt,3) ~= wpt(idx_wpt-1,3)) && (wpt(idx_wpt-1,3) ~= 0)
    for idx = 1 : length (vrt(:,1))
        if vrt(idx,3) == wpt(idx_wpt-1,3)
            vrt(idx,5) = 1;
        end
    end
end