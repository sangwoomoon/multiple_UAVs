function [wpt_tmp,det_tmp_exist,wpt_tmp_2,value] = find_waypoint(OBS_NUM,OBS_VRT,v,meet_pts,wpt_prev,p_f)

% Find a waypoint which satisfies those conditions for this algorithm.
%  First of all, we should select the adequate meeting point among meeting
%  points which are solved by "find_meeting_points.m" file. The point is
%  the closest point from the previous waypoint.
%
%  By using this meeting point, find the new waypoint which satisfies that
%  cost function.
%
%  min J = dist(vertex of obstacle,final point) + dist(vertex of obstacle,previous waypoint)
%
%  However, the path can be overlap the one obstacle which is handled on
%  this procedure. To check I used the "INPOLYGON" function to simplify the
%  problem. In that case, the waypoint is converted to the neighbor vertex.
%  there is 2 neighbor vertices in 2-D situation, and the real waypoints
%  is selected between those vertices by given cost function which is
%  written above.
%
%%%% REVISION NOTE %%%
%
% REVISION VERSION : 1.10b
% 1ST REVISION DATE : 2009. 11. 2
% 2ND REVISION DATE : 2009. 11. 5
% REVISED BY SANGWOO MOON.
%
% REVISION VERSION : 1.50
% REVISION DATE : 2010. 3. 31
% REVISED BY DAESUNG JANG.


% meeting point filtering :
%  there is a point which is the same as the previous meeting point.
%  we should erase this point.
idx_tmp = 1;
det_tmp_exist = 0;
for idx = 1 : length(meet_pts(:,1))
    if abs(meet_pts(idx,1)-wpt_prev(1,1)) > 0.01 && abs(meet_pts(idx,2)-wpt_prev(1,2)) > 0.01 % in mathematical represent, a meet_pts is not equal to final waypoint.
        tmp(idx_tmp,:) = meet_pts(idx,:);
        idx_tmp = idx_tmp + 1;
        det_tmp_exist = 1;
    end
end

if (det_tmp_exist == 1)
    clear meet_pts;
    meet_pts = tmp;

    % Real meeting point searching :
    %  find shortest meeting point among filtered points.
    min_meet_pt = meet_pts(1,:);
    for i = 2 : length(meet_pts(:,1))
        if (distance(wpt_prev(1,[1,2]),min_meet_pt) >= distance(wpt_prev(1,[1,2]),meet_pts(i,:)))
            min_meet_pt = meet_pts(i,:);
        end
    end

    % Find New Waypoint :
    %  cost function : dist(Goal pt, new WP) + dist(current WP, new WP)
    %  select the new waypoint between the neighbor vertices using the cost
    %  function. However, if the configuration space is overlapped, we have
    %  to avoid to select this point which is on that space. To avoid, I
    %  use just one "if" discriminent.

    if (min_meet_pt(1,3) <= OBS_NUM)
        cost_1 = distance(v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),:),p_f) + distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),:));
        cost_2 = distance(v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,:),p_f) + distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,:));
        value_1 = distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),:));
        value_2 = distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,:));
        if (cost_1 > cost_2)
            if (v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,5) ~= 1) % if this point is not inside in the configuration space
                wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,1:4);
                value(1,1) = value_2;
                wpt_tmp_2 = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),1:4);
                value(1,2) = value_1;
            else
                wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),1:4);
                value(1,1) = value_1;
                wpt_tmp_2 = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,1:4);
                value(1,2) = inf;
            end
        else
            if (v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),5) ~= 1)
                wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),1:4);
                value(1,1) = value_1;
                wpt_tmp_2 = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,1:4);
                value(1,2) = value_2;
            else
                wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4)+1,1:4);
                value(1,1) = value_2;
                wpt_tmp_2 = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min_meet_pt(1,4),1:4);
                value(1,2) = inf;
            end
        end

        % Check whether the path passes through the obstacle :
        %  if the solved waypoint is on the across the obstacle so that the path
        %  goes through this obstacle, it has the critical problem and we should
        %  modify this algorithm.
        %  To check, we'll use the point which is on the path solved by this
        %  algorithm. If this point is inside of the obstacle, we should change the
        %  solved waypoint to a neighbor vertex which minimize the given cost
        %  function.
        if (wpt_tmp(1,3) == wpt_prev(1,3)) && (abs(wpt_tmp(1,4)-wpt_prev(1,4)) > 1) % if two waypoints are on the same obstacle and those waypoints are not neighberhood
            CHK_PT = [(wpt_prev(1,1)+wpt_tmp(1,1))/2, (wpt_prev(1,2)+wpt_tmp(1,2))/2];
            % To simplify, I use INPOLYGON function which is a basic function in
            % MATLAB. if this code is converted into C++, some modification or
            % recongnization for this function will be required.
            % However, if the configuration space is overlapped, we have
            % to avoid to select this point which is on that space. To avoid, I
            % use just one "if" discriminent.
            if (inpolygon(CHK_PT(1,1),CHK_PT(1,2),v((OBS_VRT+1)*(wpt_tmp(1,3)-1)+1:(OBS_VRT+1)*wpt_tmp(1,3),1),v((OBS_VRT+1)*(wpt_tmp(1,3)-1)+1:(OBS_VRT+1)*wpt_tmp(1,3),2)))
                %             for idx = 1 : OBS_VRT
                %                 if abs(wpt_prev(1,1)-v((OBS_VRT+1)*(wpt_tmp(1,3)-1)+idx,1)) < 0.01 ...
                %                         && abs(wpt_prev(1,2)-v((OBS_VRT+1)*(wpt_tmp(1,3)-1)+idx,2)) < 0.01 % in mathematical represent, a vertex is equal to a previous waypoint.
                if wpt_prev(1,4) == 1
                    min = OBS_VRT;
                    max = 2;
                else
                    min = wpt_prev(1,4)-1;
                    max = wpt_prev(1,4)+1;
                end

                cost_1 = distance(v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,:),p_f) + distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,:));
                cost_2 = distance(v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,:),p_f) + distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,:));
                value_1 = distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,:));
                value_2 = distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,:));

                % change the waypoint
                if (cost_1 > cost_2)
                    if (v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,5) ~= 1) % if this point is not inside in the configuration space
                        wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,1:4);
                        value(1,1) = value_2;
                    else
                        wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,1:4);
                        value(1,1) = value_1;
                    end
                else
                    if (v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,5) ~= 1)
                        wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,1:4);
                        value(1,1) = value_1;
                    else
                        wpt_tmp(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,1:4);
                        value(1,1) = value_2;
                    end
                end
                %                 end
                %             end
            end
        end

        %%%

        if (wpt_tmp_2(1,3) == wpt_prev(1,3)) && (abs(wpt_tmp_2(1,4)-wpt_prev(1,4)) > 1) % if two waypoints are on the same obstacle and those waypoints are not neighberhood
            CHK_PT = [(wpt_prev(1,1)+wpt_tmp_2(1,1))/2, (wpt_prev(1,2)+wpt_tmp_2(1,2))/2];
            % To simplify, I use INPOLYGON function which is a basic function in
            % MATLAB. if this code is converted into C++, some modification or
            % recongnization for this function will be required.
            % However, if the configuration space is overlapped, we have
            % to avoid to select this point which is on that space. To avoid, I
            % use just one "if" discriminent.
            if (inpolygon(CHK_PT(1,1),CHK_PT(1,2),v((OBS_VRT+1)*(wpt_tmp_2(1,3)-1)+1:(OBS_VRT+1)*wpt_tmp_2(1,3),1),v((OBS_VRT+1)*(wpt_tmp_2(1,3)-1)+1:(OBS_VRT+1)*wpt_tmp_2(1,3),2)))
                %             for idx = 1 : OBS_VRT
                %                 if abs(wpt_prev(1,1)-v((OBS_VRT+1)*(wpt_tmp(1,3)-1)+idx,1)) < 0.01 ...
                %                         && abs(wpt_prev(1,2)-v((OBS_VRT+1)*(wpt_tmp(1,3)-1)+idx,2)) < 0.01 % in mathematical represent, a vertex is equal to a previous waypoint.
                if wpt_prev(1,4) == 1
                    min = OBS_VRT;
                    max = 2;
                else
                    min = wpt_prev(1,4)-1;
                    max = wpt_prev(1,4)+1;
                end

                cost_1 = distance(v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,:),p_f) + distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,:));
                cost_2 = distance(v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,:),p_f) + distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,:));
                value_1 = distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,:));
                value_2 = distance(wpt_prev(1,1:2),v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,:));

                % change the waypoint
                if (cost_1 > cost_2)
                    if (v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,5) ~= 1) % if this point is not inside in the configuration space
                        wpt_tmp_2(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,1:4);
                        value(1,1) = value_2;
                    else
                        wpt_tmp_2(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,1:4);
                        value(1,1) = value_1;
                    end
                else
                    if (v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,5) ~= 1)
                        wpt_tmp_2(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+min,1:4);
                        value(1,1) = value_1;
                    else
                        wpt_tmp_2(1,:) = v((OBS_VRT+1)*(min_meet_pt(1,3)-1)+max,1:4);
                        value(1,1) = value_2;
                    end
                end
                %                 end
                %             end
            end
        end
        
    % cluttered case.
    else
        % find the vertex which we want to use.
        for idx_vrt = OBS_NUM*(OBS_VRT+1)+1 : 2 : length(v(:,1))-1
            if (v(idx_vrt,3) == min_meet_pt(1,3)) && (v(idx_vrt,4) == min_meet_pt(1,4))
                cost_1 = distance(v(idx_vrt,:),p_f) + distance(wpt_prev(1,1:2),v(idx_vrt,:));
                cost_2 = distance(v(idx_vrt+1,:),p_f) + distance(wpt_prev(1,1:2),v(idx_vrt+1,:));
                value_1 = distance(wpt_prev(1,1:2),v(idx_vrt,:));
                value_2 = distance(wpt_prev(1,1:2),v(idx_vrt+1,:));
                if (cost_1 > cost_2)
                    if (v(idx_vrt+1,5) ~= 1) % if this point is not inside in the configuration space
                        wpt_tmp(1,:) = v(idx_vrt+1,1:4);
                        value(1,1) = value_2;
                        wpt_tmp_2 = v(idx_vrt,1:4);
                        value(1,2) = value_1;
                    else
                        wpt_tmp(1,:) = v(idx_vrt,1:4);
                        value(1,1) = value_1;
                        wpt_tmp_2 = v(idx_vrt+1,1:4);
                        value(1,2) = inf;
                    end
                else
                    if (v(idx_vrt,5) ~= 1)
                        wpt_tmp(1,:) = v(idx_vrt,1:4);
                        value(1,1) = value_1;
                        wpt_tmp_2 = v(idx_vrt+1,1:4);
                        value(1,2) = value_2;
                    else
                        wpt_tmp(1,:) = v(idx_vrt+1,1:4);
                        value(1,1) = value_2;
                        wpt_tmp_2 = v(idx_vrt,1:4);
                        value(1,2) = inf;
                    end
                end
            end
        end

        % to prevent making a path through the obstacle,
        % this procedure makes the 'DEAD ZONE', IT SHOULD BE MODIFIEDED !!!
        chk_pt = ones(1,3);
        idx_pt = zeros(1,2);
        for chk_iter = OBS_NUM*(OBS_VRT+1)+1 : length(v(:,1))
            if (v(chk_iter,1) == wpt_tmp(1,1)) && (v(chk_iter,2) == wpt_tmp(1,2))
                chk_pt(1) = chk_pt(1) + 1;
                chk_pt(chk_pt(1)) = chk_iter;
            end
        end
        if (chk_pt(1) == 3)
            wpt_temp(1,:) = wpt_tmp;
            for iter = 2 : chk_pt(1)
                if (v(chk_pt(iter)-1,3) == v(chk_pt(iter),3)) && (v(chk_pt(iter)-1,4) == v(chk_pt(iter),4))
                    idx_pt(iter-1) = chk_pt(iter)-1;
                elseif (v(chk_pt(iter)+1,3) == v(chk_pt(iter),3)) && (v(chk_pt(iter)+1,4) == v(chk_pt(iter),4))
                    idx_pt(iter-1) = chk_pt(iter)+1;
                end
            end
            cost_1 = distance(v(idx_pt(1),:),p_f) + distance(wpt_prev(1,1:2),v(idx_pt(1),:));
            cost_2 = distance(v(idx_pt(2),:),p_f) + distance(wpt_prev(1,1:2),v(idx_pt(2),:));
            value_1 = distance(wpt_prev(1,1:2),v(idx_pt(1),:));
            value_2 = distance(wpt_prev(1,1:2),v(idx_pt(2),:));
            % change the waypoint
            if (cost_1 > cost_2)
                if (v(idx_pt(2),5) ~= 1) % if this point is not inside in the configuration space
                    wpt_tmp(1,:) = v(idx_pt(2),1:4);
                    value(1,1) = value_2;
                else
                    wpt_tmp(1,:) = v(idx_pt(1),1:4);
                    value(1,1) = value_1;
                end
            else
                if (v(idx_pt(1),5) ~= 1)
                    wpt_tmp(1,:) = v(idx_pt(1),1:4);
                    value(1,1) = value_1;
                else
                    wpt_tmp(1,:) = v(idx_pt(2),1:4);
                    value(1,1) = value_2;
                end
            end
            
            % if the changed waypoint makes the path to go through the
            % obstacle, this procedure shouldn't be happened.
            [meet_pts,det_meet_pt] = find_meeting_points(OBS_NUM,OBS_VRT,v,wpt_tmp,wpt_prev);
            if (det_meet_pt)
                wpt_tmp = wpt_temp;
            end
        end
    end

else
    wpt_tmp = zeros(1,4); % initialize because it should be returned into the "find_path.m" even if there is NO waypoints after the calculation.
    value(1,1) = inf;
    wpt_tmp_2 = zeros(1,4);
    value(1,2) = inf;
end
