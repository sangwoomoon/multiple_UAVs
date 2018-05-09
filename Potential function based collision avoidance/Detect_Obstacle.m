function [scan_pts,idx] = Detect_Obstacle(P,OBS_Vert,Para_Plane,NUM_OBS,OBS_Height,Search_range,range_theta,range_psi,d_angle)

% Euler Transformtion and Acquire the Scanned Range for Each Scanning Point
D2R = pi/180;

idx = 0;
idx_pt = 0;
scan_pts = zeros(1,3);

euler = Calculate_Euler(P(4:6));

for theta = range_theta(1)*D2R : d_angle: range_theta(2)*D2R
    for psi = range_psi(1)*D2R : d_angle : range_psi(2)*D2R
        idx_pt = idx_pt + 1;        
        pt(idx_pt,1:3) = P(1:3) + [Search_range*sin(theta)*sin(psi) -Search_range*sin(theta)*cos(psi) -Search_range*cos(theta)]*euler';  %  NED Coordinate
    end
end

% Obstacle Detection%%

for iter_pt = 1 : idx_pt
    idx_detect_pts = 0;
    for iter_obs = 1 : NUM_OBS

        % Upper detection (all values are based on NED coordinate system)
        OBS_Vert_tmp = [OBS_Vert(4*(iter_obs-1)+1:4*iter_obs,:); OBS_Vert(4*(iter_obs-1)+1,:)]; % vertices setting
        OBS_Vert_lower = [OBS_Vert_tmp zeros(length(OBS_Vert_tmp(:,1)),1)];
        OBS_Vert_upper = [OBS_Vert_tmp -OBS_Height(iter_obs)*ones(length(OBS_Vert_tmp(:,1)),1)];
        t = (-Para_Plane(6*(iter_obs-1)+2,4)-P(1:3)*(Para_Plane(6*(iter_obs-1)+2,1:3))')/((pt(iter_pt,1:3)-P(1:3))*(Para_Plane(6*(iter_obs-1)+2,1:3))');

        if t > 0 && t < 1
            scan_pts_tmp = P(1:3)+t*(pt(iter_pt,1:3)-P(1:3));
            if (inpolygon(scan_pts_tmp(1,1),scan_pts_tmp(1,2),OBS_Vert_upper(:,1),OBS_Vert_upper(:,2)))
                idx_detect_pts = idx_detect_pts + 1;
                detect_pts_tmp(idx_detect_pts,:) = scan_pts_tmp;
            end
        end

        % Side detection
        for index = 1:length(OBS_Vert_tmp)-1
            temp_vert = [OBS_Vert_lower(index,:); OBS_Vert_lower(index+1,:); OBS_Vert_upper(index+1,:); OBS_Vert_upper(index,:)];
            t = (-Para_Plane(6*(iter_obs-1)+index+2,4)-P(1:3)*(Para_Plane(6*(iter_obs-1)+index+2,1:3))')/((pt(iter_pt,1:3)-P(1:3))*(Para_Plane(6*(iter_obs-1)+index+2,1:3))');

            if t > 0 && t < 1
                scan_pts_tmp = P(1:3)+t*(pt(iter_pt,1:3)-P(1:3));
                if (inpolygon(scan_pts_tmp(1,1),scan_pts_tmp(1,3),temp_vert(:,1),temp_vert(:,3))) && (inpolygon(scan_pts_tmp(1,2),scan_pts_tmp(1,3),temp_vert(:,2),temp_vert(:,3)))
                    idx_detect_pts = idx_detect_pts + 1;
                    detect_pts_tmp(idx_detect_pts,:) = scan_pts_tmp;
                end
            end
        end
    end

    if (idx_detect_pts == 1)
        idx = idx + 1;        
        scan_pts(idx,:)= detect_pts_tmp;
    elseif (idx_detect_pts > 1)
        idx = idx + 1;
        scan_pts(idx,:) = detect_pts_tmp(1,:);
        for iter_scan = 2 : idx_detect_pts
            if distance(P(1:3),scan_pts(idx,:)) > distance(P(1:3),detect_pts_tmp(idx_detect_pts,:))
                scan_pts(idx,:) = detect_pts_tmp(idx_detect_pts,:);  % NED coordinate
            end
        end
    end
    clear detect_pts_tmp;
end