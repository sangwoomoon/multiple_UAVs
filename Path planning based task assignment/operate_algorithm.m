function [wpt,second_choice,point_res] = operate_algorithm(OBS_NUM,OBS_VRT,wpt,vrt_config,vrt_config_org,p_i,p_f,idx_wpt)

% Operate main algorithm. 
%  this algorithm is for totally known environment
%  which vertices of obstacle are known. this file gives the whole waypoints
%  which starts from the starting point to the goal point. 
%
%  The type of waypoints are as follows.
%
%  [wpt_(x_coordinate), wpt_(y_coordinate), index of obstacle, index of vertex]
%
%  The starting point and goal point are also in the member of waypoints.
%  The output data is "wpt", and the third and fourth column of those two
%  points are all 0.
%
%  In this version (1.15), point_value is used as the waypoints. to
%  construct that variable, a fundamental concept for A STAR algorithm was
%  used.
%
%  The type of waypoints are as follows.
%
%  [wpt_(x_coordinate), wpt_(y_coordinate), index of obstacle, index of vertex, cost, father_index, already_done_point, tree_depth]
%
%%%% REVISION NOTE %%%
%
% REVISION VERSION : 1.10
% REVISION DATE : 2009. 11. 3
% REVISED BY SANGWOO MOON.
%
% REVISION VERSION : 1.50
% REVISION DATE : 2010. 3. 31
% REVISED BY DAESUNG JANG.


% Waypoint Setting :
%  set up the first waypoints which is the starting point or passing point.
wpt(idx_wpt,1:4) = [p_i,0,0];
point_value = [wpt(idx_wpt,1:4) 0 0 0 1]; %[1~4:wpt, 5:cost, 6:father_index, 7:already_done_point, 8:tree_depth]
father = 1;
% Path Planning :
%  this procedure is the core of this algorithm. First of all, find the
%  meeting point which is solved by line-line meeting equation.
%  find_meeting_points.m file gives the meeting point and the discriminent
%  whether there is any meeting points. If there is meeting points, find
%  the actual waypoint by using given cost function. However, this
%  waypoint cannot be the admissible, so that check_through_obstacle.m and
%  check_fibonacci_filtering.m are operated. This algorithm is operated
%  until the path finds the final point.
while (1)
    min_v = inf;
    for id = 1:size(point_value,1)
        if point_value(id,7) < 1
            if point_value(id,5)+norm(point_value(id,1:2)-p_f) < min_v
                father = id;
                min_v = point_value(id,5)+norm(point_value(id,1:2)-p_f);
            end
        end
    end
    min_v; %%%
    point_value(father,7) = 1;
    point_value;
        
    % Find Meeting Point :
    %  find the meeting point. this meeting points cover all of the
    %  obstacle, Therefore, the number of points should be greater than 2.
    %  otherwise, there is no meeting point.
    
    % point_value(father,1:4);
    [meet_pts,det_meet_pt] = find_meeting_points(OBS_NUM,OBS_VRT,vrt_config,p_f,point_value(father,1:4));
    if (det_meet_pt)
        % Find the real meeting point and temperary waypoint :
        %  find the real meeting point which is the closest point from the
        %  previous waypoint. By given cost function, a waypoint which is
        %  temperary can be defined.
        [wpt_tmp,det_waypoint_exist,wpt_tmp_2,value] = find_waypoint(OBS_NUM,OBS_VRT,vrt_config,meet_pts,point_value(father,1:4),p_f);
        wpt_tmp; %%%
        
        %%%
        
        toggle = 0;
        for id = 1:size(point_value,1)
            if norm(point_value(id,1:2)-wpt_tmp(1,1:2))<1e-3 && point_value(id,5)<=point_value(father,5)+value(1,1)
                toggle = 1;
            end
        end
        if toggle == 1
            point_value(end+1,:) = [wpt_tmp inf father 0 point_value(father,8)+1];
        else
            point_value(end+1,:) = [wpt_tmp point_value(father,5)+value(1,1) father 0 point_value(father,8)+1];
        end
        toggle = 0;
        for id = 1:size(point_value,1)
            if norm(point_value(id,1:2)-wpt_tmp_2(1,1:2))<1e-3 && point_value(id,5)<=point_value(father,5)+value(1,2)
                toggle = 1;
            end
        end
        if toggle == 1
            point_value(end+1,:) = [wpt_tmp_2 inf father 0 point_value(father,8)+1];
        else
            point_value(end+1,:) = [wpt_tmp_2 point_value(father,5)+value(1,2) father 0 point_value(father,8)+1];
        end
                       
        %%%
        
        first_id = size(point_value,1)-1;
        second_id = size(point_value,1);
        if (det_waypoint_exist)
            % Check whether new path through the obstacle :
            %  Although the new path is optimal, this path cannot be
            %  admissible. If this path through another obstacle, we should
            %  consider that obstacle as well. This procedure is iterated
            %  until there are no obstacles on the new path.
            N_new_point = 2;

            value(1,2) = 0;
            
            %%% SHOULD BE MODIFIED...
            while (value(1,2) < inf && check_through_obstacle(wpt_tmp,point_value(father,1:4),point_value(:,1:4),OBS_NUM,OBS_VRT,vrt_config_org))
                % '1' %%%
                while (check_through_obstacle(wpt_tmp,point_value(father,1:4),point_value(:,1:4),OBS_NUM,OBS_VRT,vrt_config_org))
                    [meet_pts,det_meet_pts] = find_meeting_points(OBS_NUM,OBS_VRT,vrt_config_org,wpt_tmp,point_value(father,1:4));
                    [wpt_tmp,det_waypoint_exist,wpt_tmp_3,value] = find_waypoint(OBS_NUM,OBS_VRT,vrt_config,meet_pts,point_value(father,1:4),wpt_tmp);
                    % '1-1' %%%
                end
                
                %%%
                toggle2 = [0 0];
                toggle = 0;
                for id = 1:size(point_value,1)
                    if norm(point_value(id,1:2)-wpt_tmp(1,1:2))<1e-3 && point_value(id,5)<=point_value(father,5)+value(1,1)
                        toggle = 1;
                        toggle2(1,1) = 1; 
                    end
                end
                if toggle == 1
                    point_value(first_id,:) = [wpt_tmp inf father 0 point_value(father,8)+1];
                else
                    point_value(first_id,:) = [wpt_tmp point_value(father,5)+value(1,1) father 0 point_value(father,8)+1];
                end
                toggle = 0;
                for id = 1:size(point_value,1)
                    if norm(point_value(id,1:2)-wpt_tmp_3(1,1:2))<1e-3 && point_value(id,5)<=point_value(father,5)+value(1,2)
                        toggle = 1;
                        toggle2(1,2) = 1; 
                    end
                end
                if toggle == 1
                    point_value(end+1,:) = [wpt_tmp_3 inf father 0 point_value(father,8)+1];
                else
                    point_value(end+1,:) = [wpt_tmp_3 point_value(father,5)+value(1,2) father 0 point_value(father,8)+1];
                end

                %%%

                wpt_tmp = wpt_tmp_3;
                first_id = size(point_value,1);
                N_new_point = N_new_point + 1;

                if norm(toggle2) > 0
                    break;
                end

            end

            if point_value(second_id,5) < inf
                value(1,2) = 0;
                
                %%% SHOULD BE MODIFIED...
                while (value(1,2) < inf && check_through_obstacle(wpt_tmp_2,point_value(father,1:4),point_value(:,1:4),OBS_NUM,OBS_VRT,vrt_config_org))
                    %'2' %%%
                    while (check_through_obstacle(wpt_tmp_2,point_value(father,1:4),point_value(:,1:4),OBS_NUM,OBS_VRT,vrt_config_org))
                        %'2-1' %%%
                        [meet_pts,det_meet_pts] = find_meeting_points(OBS_NUM,OBS_VRT,vrt_config_org,wpt_tmp_2,point_value(father,1:4));
                        [wpt_tmp_2,det_waypoint_exist,wpt_tmp_3,value] = find_waypoint(OBS_NUM,OBS_VRT,vrt_config,meet_pts,point_value(father,1:4),wpt_tmp_2);
                    end

                    %%%
                    
                    toggle2 = [0 0];
                    toggle = 0;
                    for id = 1:size(point_value,1)
                        if norm(point_value(id,1:2)-wpt_tmp_2(1,1:2))<1e-3 && point_value(id,5)<=point_value(father,5)+value(1,1)
                            toggle = 1;
                            toggle2(1,1) = 1; 
                        end
                    end
                    if toggle == 1
                        point_value(second_id,:) = [wpt_tmp_2 inf father 0 point_value(father,8)+1];
                    else
                        point_value(second_id,:) = [wpt_tmp_2 point_value(father,5)+value(1,1) father 0 point_value(father,8)+1];
                    end
                    toggle = 0;
                    for id = 1:size(point_value,1)
                        if norm(point_value(id,1:2)-wpt_tmp_3(1,1:2))<1e-3 && point_value(id,5)<=point_value(father,5)+value(1,2)
                            toggle = 1;
                            toggle2(1,2) = 1; 
                        end
                    end
                    if toggle == 1
                        point_value(end+1,:) = [wpt_tmp_3 inf father 0 point_value(father,8)+1];
                    else
                        point_value(end+1,:) = [wpt_tmp_3 point_value(father,5)+value(1,2) father 0 point_value(father,8)+1];
                    end

                    %%%
                    
                    wpt_tmp_2 = wpt_tmp_3;
                    second_id = size(point_value,1);
                    N_new_point = N_new_point + 1;
                    %%%toggle2
                    if norm(toggle2) > 0
                       break;
                    end
                    
                end
            end
            % Fibonacci filtering :
            %  There is the three neighboring waypoints that the middle
            %  waypoint can be cancelled out because the path which
            %  connects between the other waypoints are admissible. By the
            %  optimal principle, there is better path which cancels the
            %  middle waypoint. Fibonacci filtering, which is named from
            %  Fibonacci series, checks this situation and cancels that
            %  waypoint. This procedure is iterated until there are no
            %  situations for the new path.
            
            if (point_value(father,8) >= 2)
                original_father = father;

                for id_temp = 1:N_new_point
                    if point_value(end+1-id_temp,5) < inf
                        father = original_father;
                        while ~(check_fibonacci_filtering(point_value(end+1-id_temp,1:4),point_value(point_value(father,6),1:4),point_value(1:end-id_temp,1:4),OBS_NUM,OBS_VRT,vrt_config))
                            %%%'3'
                            father = point_value(father,6);
                            point_value(end+1-id_temp,:) = [point_value(end+1-id_temp,1:4) point_value(father,5)+norm(point_value(father,1:2)-point_value(end+1-id_temp,1:2)) father 0 point_value(father,8)+1];
                            if point_value(father,8) < 2
                                break;
                            end
                        end
                    end
                end
                
            end
            
            % After having those two filtering process, the waypoint is the
            % real and admissible point so that we can use this obviously.
            % plot(wpt(idx_wpt,1),wpt(idx_wpt,2),'LineStyle','o','color',rand(1,3));
            clear meet_pts det_meet_pt;
        else
            point_value(end+1,:) = [p_f 0 0 point_value(father,5)+norm(point_value(father,1:2)-p_f) father 0 point_value(father,8)+1];
            break;
        end

    else
        point_value(end+1,:) = [p_f 0 0 point_value(father,5)+norm(point_value(father,1:2)-p_f) father 0 point_value(father,8)+1];
        break;
    end
end
id = size(point_value,1);
id2 = 1;
while(point_value(id,6)>0)
    temp_pt(id2,:) = point_value(id,1:4);
    id2  = id2 + 1;
    id = point_value(id,6);
end
for id = 1:size(temp_pt,1)
    wpt(idx_wpt+id,1:4)=temp_pt(end+1-id,:);
end
min_temp = inf;
check_2 = 0;
check_3 = 0;
for id = 1:size(point_value,1)
    if point_value(id,8) == 2
        check_2 = 1;
        if norm(point_value(id,1:2)-wpt(2,1:2))>1e-3 && point_value(id,5)+norm(point_value(id,1:2)-p_f)<min_temp
            check_3 = 1;
            min_temp = point_value(id,5)+norm(point_value(id,1:2)-p_f);
            id_min = id;
        end
    end
end
if check_2 == 1 && check_3 == 1
    second_choice = point_value(id_min,1:4);
else
    second_choice = wpt(2,1:4);
end

point_res = point_value(length(point_value(:,1)),5);