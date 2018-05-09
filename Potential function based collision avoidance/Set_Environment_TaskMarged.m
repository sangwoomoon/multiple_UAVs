function [OBS_Vert,Para_Plane] = Set_Environment_TaskMarged(NUM_OBS,OBS_Height,OBS_VRT)

figure(1)
grid on
hold on
axis equal tight
axis([-110 110 -110 110 0 50]);

% obstacle setting

for iter_obs = 1 : 10 
    for iter_vert = 1 : 4
        OBS_Vert(4*(iter_obs-1)+iter_vert,1) = OBS_VRT(5*(iter_obs-1)+iter_vert,1);
        OBS_Vert(4*(iter_obs-1)+iter_vert,2) = OBS_VRT(5*(iter_obs-1)+iter_vert,2);  % matlab coordinate
        OBS_Vert(4*(iter_obs-1)+iter_vert,2) = -OBS_Vert(4*(iter_obs-1)+iter_vert,2);  % NED coordinate
        % plot(OBS_Vert(4*(iter_obs-1)+iter_vert,1), -OBS_Vert(4*(iter_obs-1)+iter_vert,2),'g.');
    end

    % 3 dimensional obstacle setting
    OBS_Vert_tmp = [OBS_Vert(4*(iter_obs-1)+1:4*iter_obs,:); OBS_Vert(4*(iter_obs-1)+1,:)]; % vertices setting
    clr = rand(1,3);
    OBS_Vert_lower = [OBS_Vert_tmp zeros(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate
    OBS_Vert_upper = [OBS_Vert_tmp -OBS_Height(iter_obs)*ones(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate

    patch(OBS_Vert_lower(:,1), -OBS_Vert_lower(:,2), -OBS_Vert_lower(:,3), clr);  % matlab coordinate
    patch(OBS_Vert_upper(:,1), -OBS_Vert_upper(:,2), -OBS_Vert_upper(:,3), clr);  % matlab coordinate
    
    % Define the Equations of Upper and Lower Planes
    Para_Plane(6*(iter_obs-1)+1,:) = null([OBS_Vert_lower(1:4,:) ones(4,1)])';  % NED coordinate
    Para_Plane(6*(iter_obs-1)+2,:) = null([OBS_Vert_upper(1:4,:) ones(4,1)])';  % NED coordinate

    for index = 1:length(OBS_Vert_tmp)-1
        temp_vert = [OBS_Vert_lower(index,:); OBS_Vert_lower(index+1,:); OBS_Vert_upper(index+1,:); OBS_Vert_upper(index,:)];  % NED coordinate
        Para_Plane(6*(iter_obs-1)+index+2,:) = null([temp_vert(1:4,:) ones(4,1)])';  % NED coordinate
        patch(temp_vert(:,1), -temp_vert(:,2), -temp_vert(:,3),clr);  % matlab coordinate
    end
    
    %text(OBS_Vert(4*(iter_obs-1)+1,1),-OBS_Vert(4*(iter_obs-1)+1,2), OBS_Height(iter_obs), sprintf('%d',iter_obs));  % matlab coordinate
end

for iter_obs = 11 : 15
    for iter_vert = 1 : 4
        [OBS_Vert(4*(iter_obs-1)+iter_vert,1),OBS_Vert(4*(iter_obs-1)+iter_vert,2)]=ginput(1);
        OBS_Vert(4*(iter_obs-1)+iter_vert,2) = -OBS_Vert(4*(iter_obs-1)+iter_vert,2);  % NED coordinate
        % plot(OBS_Vert(4*(iter_obs-1)+iter_vert,1), -OBS_Vert(4*(iter_obs-1)+iter_vert,2),'g.');
    end

    % 3 dimensional obstacle setting
    OBS_Vert_tmp = [OBS_Vert(4*(iter_obs-1)+1:4*iter_obs,:); OBS_Vert(4*(iter_obs-1)+1,:)]; % vertices setting
    clr = rand(1,3);
    OBS_Vert_lower = [OBS_Vert_tmp zeros(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate
    OBS_Vert_upper = [OBS_Vert_tmp -OBS_Height(iter_obs)*ones(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate

    patch(OBS_Vert_lower(:,1), -OBS_Vert_lower(:,2), -OBS_Vert_lower(:,3), clr);  % matlab coordinate
    patch(OBS_Vert_upper(:,1), -OBS_Vert_upper(:,2), -OBS_Vert_upper(:,3), clr);  % matlab coordinate
    
    % Define the Equations of Upper and Lower Planes
    Para_Plane(6*(iter_obs-1)+1,:) = null([OBS_Vert_lower(1:4,:) ones(4,1)])';  % NED coordinate
    Para_Plane(6*(iter_obs-1)+2,:) = null([OBS_Vert_upper(1:4,:) ones(4,1)])';  % NED coordinate

    for index = 1:length(OBS_Vert_tmp)-1
        temp_vert = [OBS_Vert_lower(index,:); OBS_Vert_lower(index+1,:); OBS_Vert_upper(index+1,:); OBS_Vert_upper(index,:)];  % NED coordinate
        Para_Plane(6*(iter_obs-1)+index+2,:) = null([temp_vert(1:4,:) ones(4,1)])';  % NED coordinate
        patch(temp_vert(:,1), -temp_vert(:,2), -temp_vert(:,3),clr);  % matlab coordinate
    end
    
    %text(OBS_Vert(4*(iter_obs-1)+1,1),-OBS_Vert(4*(iter_obs-1)+1,2), OBS_Height(iter_obs), sprintf('%d',iter_obs));  % matlab coordinate
end