function [landmark,get_first] = set_landmark(meet_pts,get_first,COV_RANGE)

% landmark setting for scanned points.
idx_landmark = 1;
landmark(idx_landmark,:) = meet_pts(1,:);
idx_landmark = idx_landmark+1;

for iter = 2 : length(meet_pts(:,1))-1
    chk_slp(1,1) = atan2(meet_pts(iter,2)-meet_pts(iter-1,2),meet_pts(iter,1)-meet_pts(iter-1,1));       
    chk_slp(1,2) = atan2(meet_pts(iter+1,2)-meet_pts(iter,2),meet_pts(iter+1,1)-meet_pts(iter,1));
    if (abs(chk_slp(1,1) - chk_slp(1,2)) > 0.1)
        if (distance(meet_pts(iter+1,:),meet_pts(iter,:)) < 2*COV_RANGE)
            landmark(idx_landmark,:) = meet_pts(iter,:);
            idx_landmark = idx_landmark+1;                
            landmark(idx_landmark,:) = meet_pts(iter,:);
            idx_landmark = idx_landmark+1;                  
         else
            landmark(idx_landmark,:) = meet_pts(iter,:);
            idx_landmark = idx_landmark+1;
            landmark(idx_landmark,:) = meet_pts(iter+1,:);
            idx_landmark = idx_landmark+1;                     
        end
    elseif distance(meet_pts(iter+1,:),meet_pts(iter,:)) > 2*COV_RANGE
        landmark(idx_landmark,:) = meet_pts(iter,:);
        idx_landmark = idx_landmark+1;                
        landmark(idx_landmark,:) = meet_pts(iter+1,:);
        idx_landmark = idx_landmark+1;
    end
end

landmark(idx_landmark,:) = meet_pts(length(meet_pts(:,1)),:);
get_first = get_first + 1;        


