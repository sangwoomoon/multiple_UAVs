function [search_pts,meet_pts,expand_pts] = data_update(meet_pts,meet_pts_tmp,cw_ccw,KNOBS_NUM,cov_range,OBS_VRT,vrt_config)

% at first, meet_pts_tmp should be stored on the meet_pts.
if (meet_pts(1,3) == 0) && (meet_pts(1,4) == 0)
    clear meet_pts;
    meet_pts = meet_pts_tmp;
else
    for iter_tmp = 1 : length(meet_pts_tmp(:,1))
        chk_iter = 0;  % index to find the same edge
        for iter_mpt = 1 : length(meet_pts(:,1))
            if (meet_pts_tmp(iter_tmp,3) == meet_pts(iter_mpt,3)) && (meet_pts_tmp(iter_tmp,4) == meet_pts(iter_mpt,4))
                chk_iter = chk_iter + 1;
                chk_idx(chk_iter) = iter_mpt;
                chk_pts(chk_iter,:) = meet_pts(iter_mpt,:);  % store the meeting point which is on the same edge (used to the data renewal).
            end
        end
        if (chk_iter == 2)
            if (meet_pts_tmp(iter_tmp,5) > max(chk_pts(:,5))) || (meet_pts_tmp(iter_tmp,5) < min(chk_pts(:,5)))
                chk_dist = [distance(meet_pts_tmp(iter_tmp,1:2),chk_pts(1,1:2)),distance(meet_pts_tmp(iter_tmp,1:2),chk_pts(2,1:2))];
                if (chk_dist(1) > chk_dist(2))
                    meet_pts(chk_idx(2),:) = meet_pts_tmp(iter_tmp,:);
                else
                    meet_pts(chk_idx(1),:) = meet_pts_tmp(iter_tmp,:);
                end
            end
        elseif (chk_iter == 1) && (chk_pts(1,1) ~= meet_pts_tmp(iter_tmp,1)) && (chk_pts(1,2) ~= meet_pts_tmp(iter_tmp,2))
            idx = length(meet_pts(:,1))+1;
            meet_pts(idx,:) = meet_pts_tmp(iter_tmp,:);
        elseif (chk_iter == 0)
            idx = length(meet_pts(:,1))+1;
            meet_pts(idx,:) = meet_pts_tmp(iter_tmp,:);
        end
    end
end

% cancel the non-paired points.
idx_new = 0;
for iter_pts = 1 : length(meet_pts(:,1))
    det = 0;
    for iter_com = 1 : length(meet_pts(:,1))
        if (meet_pts(iter_pts,3) == meet_pts(iter_com,3)) && (meet_pts(iter_pts,4) == meet_pts(iter_com,4))
            det = det + 1;
        end
    end
    if (det ~= 1)
        idx_new = idx_new + 1;
        meet_pts_new(idx_new,:) = meet_pts(iter_pts,:);
    end
end

clear meet_pts;
meet_pts = meet_pts_new;

% data sorting
meet_pts = sortrows(meet_pts,[3 4 5]);
search_pts = meet_pts;

% searched line expansion and make the part of the configuration space
% by using VECTOR equation of line.

% search_pts : original searching lines.
% meet_pts : extended searching lines to prevent from approaching very close
% to edges.
% extend_pts : expanded searching lines which informs the configuration
% space.

for iter = 1 : length(meet_pts(:,1))
    if (meet_pts(iter,5) == 0)
        expand_pts(iter,1:2) = vrt_config(((meet_pts(iter,3)-1)-KNOBS_NUM)*(OBS_VRT+1)+meet_pts(iter,4),1:2);
        if (meet_pts(iter,6) ~= -1)
            if (mod(iter,2) == 1)
                direction_vector = (meet_pts(iter,1:2)-meet_pts(iter+1,1:2));
            else
                direction_vector = (meet_pts(iter,1:2)-meet_pts(iter-1,1:2));
            end
            meet_pts(iter,1:2) = meet_pts(iter,1:2)+direction_vector/norm(direction_vector)*cov_range;
            meet_pts(iter,6) = -1;
            search_pts(iter,6) = -1;
        elseif (search_pts(iter,6) == -1)
            if (mod(iter,2) == 1)
                direction_vector = (search_pts(iter,1:2)-search_pts(iter+1,1:2));
            else
                direction_vector = (search_pts(iter,1:2)-search_pts(iter-1,1:2));
            end
            search_pts(iter,1:2) = search_pts(iter,1:2)-direction_vector/norm(direction_vector)*cov_range;
        end
    elseif (meet_pts(iter,5) == 1)
        expand_pts(iter,1:2) = vrt_config(((meet_pts(iter,3)-1)-KNOBS_NUM)*(OBS_VRT+1)+meet_pts(iter,4)+1,1:2);
        if (meet_pts(iter,6) ~= -1)
            if (mod(iter,2) == 1)
                direction_vector = (meet_pts(iter,1:2)-meet_pts(iter+1,1:2));
            else
                direction_vector = (meet_pts(iter,1:2)-meet_pts(iter-1,1:2));
            end
            meet_pts(iter,1:2) = meet_pts(iter,1:2)+direction_vector/norm(direction_vector)*cov_range;
            meet_pts(iter,6) = -1;
            search_pts(iter,6) = -1;
        elseif (search_pts(iter,6) == -1)
            if (mod(iter,2) == 1)
                direction_vector = (search_pts(iter,1:2)-search_pts(iter+1,1:2));
            else
                direction_vector = (search_pts(iter,1:2)-search_pts(iter-1,1:2));
            end
            search_pts(iter,1:2) = search_pts(iter,1:2)-direction_vector/norm(direction_vector)*cov_range;
        end
    else
        if mod(iter,2) == 1
            % same direction as the original obstacle direction(CW / CCW)
            direction_vector = (meet_pts(iter,1:2)-meet_pts(iter+1,1:2));
            if cw_ccw(meet_pts(iter,3)-KNOBS_NUM) == 0
                normal_vector(1,:) = [-direction_vector(2),direction_vector(1)];
            else
                normal_vector(1,:) = [direction_vector(2),-direction_vector(1)];
            end
            ref_pts = meet_pts(iter,1:2) + normal_vector(1,1:2)/norm(normal_vector(1,1:2))*cov_range;
            expand_pts(iter,1:2) = ref_pts+(meet_pts(iter,1:2)-meet_pts(iter+1,1:2))/norm(meet_pts(iter,1:2)-meet_pts(iter+1,1:2))*cov_range;
        else
            % opposite direction as the original obstacle direction(CW / CCW)
            direction_vector = (meet_pts(iter,1:2)-meet_pts(iter-1,1:2));
            if cw_ccw(meet_pts(iter,3)-KNOBS_NUM) == 0
                normal_vector(1,:) = [direction_vector(2),-direction_vector(1)];
            else
                normal_vector(1,:) = [-direction_vector(2),direction_vector(1)];
            end
            ref_pts = meet_pts(iter,1:2) + normal_vector(1,1:2)/norm(normal_vector(1,1:2))*cov_range;
            expand_pts(iter,1:2) = ref_pts+(meet_pts(iter,1:2)-meet_pts(iter-1,1:2))/norm(meet_pts(iter,1:2)-meet_pts(iter-1,1:2))*cov_range;
        end
    end
    expand_pts(iter,3:6) = meet_pts(iter,3:6);
end

