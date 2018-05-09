
clear all;
clc;
clf;

SCAN_RANGE = 15;
SCAN_THETA = 45*pi/180;

HLI_vel = 5; % m/s
dt = 0.2;    % Time Interval

COV_RANGE = 5;
OBS_NUM = 1;
OBS_VRT = 4;

axis([0 200 0 200])
grid on
hold on
axis equal tight

[p_i(1,1), p_i(1,2)] = ginput(1);% Initial Point
plot(p_i(:,1),p_i(:,2), 'ro');
text(p_i(:,1)-15,p_i(:,2)-5,sprintf('%s','Starting Point'));

[p_f(1,1), p_f(1,2)] = ginput(1);% Final Point
plot(p_f(:,1),p_f(:,2), 'bo');
text(p_f(:,1)-15,p_f(:,2)-5,sprintf('%s','Goal Point'));

% Obstacles
vrt = set_obstacle(OBS_NUM,OBS_VRT);

% Update current location of Helicopter

% Initial
x = p_i(1,1);
y = p_i(1,2);
angle = atan2(p_f(1,2)-p_i(1,2),p_f(1,1)-p_i(1,1));

wpt = [x, y, angle];

%%% 해야할 일
%%% scanning 할 때 꼭지점 부분 해결
%%% cover zone (on-line configration space) 이용
%%% --> 속도가 조건에 따라 변하는 거 적용.
%%% Maximum turning radius.
%%% Obstacle Data 리뉴얼 : 기존 데이터 삭제 않기.

flag_meet_pts = 0;
for i = 1 : 2
    det_mpt = 0;    
    index = 1;
    for srh_angle = -SCAN_THETA : 0.02 : SCAN_THETA
        srh_bound = [wpt(i,1),wpt(i,2); wpt(i,1)+SCAN_RANGE*cos(wpt(i,3)+srh_angle),wpt(i,2)+SCAN_RANGE*sin(wpt(i,3)+srh_angle)];
            [meet_pts_tmp,det_mpt] = find_obstacles(OBS_NUM,OBS_VRT,vrt,srh_bound(1,:),srh_bound(2,:));
        if (det_mpt == 1)
            meet_pts(index,:) = meet_pts_tmp;
            ndex = index + 1;
            flag_meet_pts = 1;
        end
    det_mpt = 0;
    end
    if (flag_meet_pts)
        plot(meet_pts(:,1),meet_pts(:,2));
    end
end