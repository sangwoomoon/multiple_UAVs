function [meet_pts,flag_meet_pts,srh_bound] = scan_obstacle(wpt,vrt,KNOBS_NUM,OBS_NUM,OBS_VRT,SCAN_RANGE,SCAN_THETA)

% meet_pts type : [x y idx_bldg idx_edge t(parameter, from lower edge # to upper edge #) index of finding boundary point]
meet_pts = [0 0 0 0 0 0];
flag_meet_pts = 0;
index = 0;
det_mpt = 0;

% scan element :: boundary points
% type [ position ; left boundary ; right boundary]
% therefore, 
% srh_bound([1,2],:) = left boundary line
% srh_bound([1,3],:) = right boundary line
% srh_bound([2,3],:) = forward curve (consider as a line)
srh_bound = [wpt(1,1),wpt(1,2); wpt(1,1)+SCAN_RANGE*cos(wpt(1,3)-SCAN_THETA),wpt(1,2)+SCAN_RANGE*sin(wpt(1,3)-SCAN_THETA);wpt(1,1)+SCAN_RANGE*cos(wpt(1,3)+SCAN_THETA),wpt(1,2)+SCAN_RANGE*sin(wpt(1,3)+SCAN_THETA)];

% search zone animation.
% for iter = 1 : length(srh_bound(:,1))-1
%     srh_line(iter) = line([srh_bound(iter,1),srh_bound(iter+1,1)],[srh_bound(iter,2),srh_bound(iter+1,2)],'color','black','EraseMode','xor');
%     set(srh_line(iter),'XData',(srh_bound(iter,1)+srh_bound(iter+1,1))/2,'YData',(srh_bound(iter,2)+srh_bound(iter+1,2))/2);
% end
% srh_line(iter+1) = line([srh_bound(1,1),srh_bound(iter+1,1)],[srh_bound(1,2),srh_bound(iter+1,2)],'color','black','EraseMode','xor');
% set(srh_line(iter),'XData',(srh_bound(1,1)+srh_bound(iter+1,1))/2,'YData',(srh_bound(1,2)+srh_bound(iter+1,2))/2);
% drawnow;

iter = 1;
for iter_1 = 1 : length(srh_bound(:,1))-1
    for iter_2 = iter_1+1 : length(srh_bound(:,1))
        [meet_pts_tmp,det_mpt] = find_obstacles(KNOBS_NUM,OBS_NUM,OBS_VRT,vrt,srh_bound(iter_1,:),srh_bound(iter_2,:),iter_2+iter_1);
        if (det_mpt == 1)
            num = length(meet_pts_tmp(:,1));
            meet_pts(index+1:index+num,:) = [meet_pts_tmp,iter*ones(num,1)];
            index = index + num;
            flag_meet_pts = 1;
        end
        det_mpt = 0;
        iter = iter + 1;
    end
end

% find the edge which is inside of the detect region.
idx = 0;
for iter_vrt = 1 : length(vrt(:,1))
    if (mod(iter_vrt,OBS_VRT+1) ~= 0)
        if (inpolygon(vrt(iter_vrt,1),vrt(iter_vrt,2),[srh_bound(:,1);srh_bound(1,1)],[srh_bound(:,2);srh_bound(1,2)]))
            idx = idx + 1;
            meet_pts(index+idx,:) = [vrt(iter_vrt,:), 0, 0];
            if (vrt(iter_vrt,4) ~= 1)
                idx = idx + 1;
                meet_pts(index+idx,:) = [vrt(iter_vrt,1:3),vrt(iter_vrt,4)-1, 1, 0];
            else
                idx = idx + 1;
                meet_pts(index+idx,:) = [vrt(iter_vrt,1:3),OBS_VRT, 1, 0];
            end
        end
    end
end

if (flag_meet_pts == 1)
    idx = 0;
    for iter_mpt = 1 : length(meet_pts(:,1))
        [garbage,det] = find_meeting_points(OBS_NUM,OBS_VRT,vrt,meet_pts(iter_mpt,1:2),wpt(1,1:2));
        if (det == 0)
            idx = idx + 1;
            meet_pts_real(idx,:) = meet_pts(iter_mpt,:);
        end
    end
    clear meet_pts;
    meet_pts = meet_pts_real;
end

