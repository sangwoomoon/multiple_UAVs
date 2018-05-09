

close all;
fig = figure(1);
%axis([-100 100 -100 100 0 50]);

set(fig,'DoubleBuffer','on');
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
hold on;

deter_movie_make = 0;  % discrimination to make a movie file. 0 : NO, 1 : YES.
det_drawUAV = 0;   % discrimination to draw the UAVs. 0 : NO, 1 : YES.
% obstacle setting

for iter_buld=1:10
    if (iter_buld<=10)
        clr_buld(iter_buld,:) = [0 1 0];
    else
        clr_buld(iter_buld,:) = [1 0 0];
    end
end

for iter_obs = 1 : NUM_OBS
    % 3 dimensional obstacle setting
    OBS_Vert_tmp = [OBS_Vert(4*(iter_obs-1)+1:4*iter_obs,:); OBS_Vert(4*(iter_obs-1)+1,:)]; % vertices setting
    OBS_Vert_lower = [OBS_Vert_tmp zeros(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate
    OBS_Vert_upper = [OBS_Vert_tmp -OBS_HEIGHT(iter_obs)*ones(length(OBS_Vert_tmp(:,1)),1)];  % NED coordinate

    if (iter_obs <= 10)
        plot3(OBS_Vert_lower(:,1), -OBS_Vert_lower(:,2), -OBS_Vert_lower(:,3), 'Color', clr_buld(iter_obs,1:3), 'Linestyle','--');  % matlab coordinate
        plot3(OBS_Vert_upper(:,1), -OBS_Vert_upper(:,2), -OBS_Vert_upper(:,3), 'Color', clr_buld(iter_obs,1:3), 'Linestyle','--');  % matlab coordinate

        for index = 1:length(OBS_Vert_tmp)-1
            temp_vert = [OBS_Vert_lower(index,:); OBS_Vert_lower(index+1,:); OBS_Vert_upper(index+1,:); OBS_Vert_upper(index,:)];  % NED coordinate
            plot3(temp_vert(:,1), -temp_vert(:,2), -temp_vert(:,3),'Color', clr_buld(iter_obs,1:3), 'Linestyle','--');  % matlab coordinate
        end
    else
        plot3(OBS_Vert_lower(:,1), -OBS_Vert_lower(:,2), -OBS_Vert_lower(:,3), 'Color', clr_buld(iter_obs,1:3), 'Linestyle','--');  % matlab coordinate
        plot3(OBS_Vert_upper(:,1), -OBS_Vert_upper(:,2), -OBS_Vert_upper(:,3), 'Color', clr_buld(iter_obs,1:3), 'Linestyle','--');  % matlab coordinate

        for index = 1:length(OBS_Vert_tmp)-1
            temp_vert = [OBS_Vert_lower(index,:); OBS_Vert_lower(index+1,:); OBS_Vert_upper(index+1,:); OBS_Vert_upper(index,:)];  % NED coordinate
            plot3(temp_vert(:,1), -temp_vert(:,2), -temp_vert(:,3),'Color', clr_buld(iter_obs,1:3), 'Linestyle','--');  % matlab coordinate
        end
    end
end

xlabel('X direction (m)');
ylabel('Y direction (m)');
zlabel('Z direction (m)');
set(gcf,'color',[1 1 1]);

% for iter = 1 : NUM_UAV
%     plot3(POS_UAV(1,6*(iter-1)+1),-POS_UAV(1,6*(iter-1)+2),-POS_UAV(1,6*(iter-1)+3),'ro');
%     plot3(POS_UAV(length(POS_UAV(:,1)),6*(iter-1)+1),-POS_UAV(length(POS_UAV(:,1)),6*(iter-1)+2),-POS_UAV(length(POS_UAV(:,1)),6*(iter-1)+3),'bo');
% %     text(POS_UAV(1,1)-5,-POS_UAV(1,1)-28,sprintf('Starting Point'));
% %     text(POS_UAV(length(POS_UAV(:,1)),1)+45,-POS_UAV(length(POS_UAV(:,1)),1)+12,sprintf('Goal Point'));
% end

if (det_drawUAV == 1)
    % Helicopter scale.
    scale = 0.25;

    %%%%%% Draw RUAV :: head + rotor + boom + tail %%%%%%%
    heli_head_1=[-2 0 -2; 5 0 2; -2 -2 2]*scale;
    heli_head_2=[-2 0 -2; 5 0 2; -2 2 2]*scale;
    heli_head_3=[-2 -2 2; -2 2 2; 5 0 2]*scale;
    heli_head_4=[-2 -2 2; -2 2 2; -2 0 -2]*scale;

    heli_boom = [-2 0 0;-10 0 0]*scale;
    heli_tail = [-10,0,0;-11,-2,0;-9,0,0;-11,2,0;-10,0,0]*scale;

    part_num = 40;
    for idx=1:part_num
        x=6*cos(2*pi/part_num*idx)*scale;
        y=6*sin(2*pi/part_num*idx)*scale;
        heli_rotor(idx,1)=x;
        heli_rotor(idx,2)=y;
        heli_rotor(idx,3)=-2*scale;
    end
    heli_rotor(part_num+1,:)=heli_rotor(1,:);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%% RUAV Assign %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for idx = 1 : NUM_UAV
        clr = rand(1,3);
        head_1(idx) = patch(heli_head_1(:,1),heli_head_1(:,2),heli_head_1(:,3),clr);
        head_2(idx) = patch(heli_head_2(:,1),heli_head_2(:,2),heli_head_2(:,3),clr);
        head_3(idx) = patch(heli_head_3(:,1),heli_head_3(:,2),heli_head_3(:,3),clr);
        head_4(idx) = patch(heli_head_4(:,1),heli_head_4(:,2),heli_head_4(:,3),clr);
        rotor(idx) = line(heli_rotor(:,1),heli_rotor(:,2),heli_rotor(:,3),'color','black');
        boom(idx) = line(heli_boom(:,1),heli_boom(:,2),heli_boom(:,3),'color','black','linewidth',2);
        tail(idx) = patch(heli_tail(:,1),heli_tail(:,2),heli_tail(:,3),rand(1,3));
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
% %%%%%%%%%%%%%%%%%%%%%% Makinng Movie Clip %%%%%%%%%%%%%%%%%%%%%%%%%%%
 if (deter_movie_make == 1)
   aviobj = avifile('simulation_scanning.avi','fps',10,'compression','IYUV','Quality',100);
 end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clr_uav = rand(NUM_UAV,3);

for iter_ANI = 1 :  length(POS_UAV(:,1))
    
    if (det_drawUAV == 1)
        for idx_UAV = 1 : NUM_UAV
            POS_UAV_X(idx_UAV) = POS_UAV(iter_ANI,6*(idx_UAV-1)+1);
            POS_UAV_Y(idx_UAV) = POS_UAV(iter_ANI,6*(idx_UAV-1)+2);
            POS_UAV_Z(idx_UAV) = POS_UAV(iter_ANI,6*(idx_UAV-1)+3);
        end

        for idx_UAV = 1 : NUM_UAV
            %%%%%%%%%%%%%%%%%%%%%%%%% Angle Assign %%%%%%%%%%%%%%%%%%%%%%%%%%%
            euler = Calculate_Euler(POS_UAV(iter_ANI,6*(idx_UAV-1)+4:6*(idx_UAV-1)+6));
            heli_head_1_trans=[heli_head_1(:,1) -heli_head_1(:,2) -heli_head_1(:,3)]*euler';
            heli_head_2_trans=[heli_head_2(:,1) -heli_head_2(:,2) -heli_head_2(:,3)]*euler';
            heli_head_3_trans=[heli_head_3(:,1) -heli_head_3(:,2) -heli_head_3(:,3)]*euler';
            heli_head_4_trans=[heli_head_4(:,1) -heli_head_4(:,2) -heli_head_4(:,3)]*euler';
            heli_boom_trans = [heli_boom(:,1) -heli_boom(:,2) -heli_boom(:,3)]*euler';
            heli_tail_trans = [heli_tail(:,1) -heli_tail(:,2) -heli_tail(:,3)]*euler';
            heli_rotor_trans = [heli_rotor(:,1) -heli_rotor(:,2) -heli_rotor(:,3)]*euler';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            set(head_1(idx_UAV),'Xdata',heli_head_1_trans(:,1)+POS_UAV_X(idx_UAV),'YData',-heli_head_1_trans(:,2)-POS_UAV_Y(idx_UAV),'ZData',heli_head_1_trans(:,3)-POS_UAV_Z(idx_UAV));
            set(head_2(idx_UAV),'Xdata',heli_head_2_trans(:,1)+POS_UAV_X(idx_UAV),'YData',-heli_head_2_trans(:,2)-POS_UAV_Y(idx_UAV),'ZData',heli_head_2_trans(:,3)-POS_UAV_Z(idx_UAV));
            set(head_3(idx_UAV),'Xdata',heli_head_3_trans(:,1)+POS_UAV_X(idx_UAV),'YData',-heli_head_3_trans(:,2)-POS_UAV_Y(idx_UAV),'ZData',heli_head_3_trans(:,3)-POS_UAV_Z(idx_UAV));
            set(head_4(idx_UAV),'Xdata',heli_head_4_trans(:,1)+POS_UAV_X(idx_UAV),'YData',-heli_head_4_trans(:,2)-POS_UAV_Y(idx_UAV),'ZData',heli_head_4_trans(:,3)-POS_UAV_Z(idx_UAV));
            set(rotor(idx_UAV),'Xdata',heli_rotor_trans(:,1)+POS_UAV_X(idx_UAV),'YData',-heli_rotor_trans(:,2)-POS_UAV_Y(idx_UAV),'ZData',heli_rotor_trans(:,3)-POS_UAV_Z(idx_UAV));
            set(boom(idx_UAV),'Xdata',heli_boom_trans(:,1)+POS_UAV_X(idx_UAV),'YData',-heli_boom_trans(:,2)-POS_UAV_Y(idx_UAV),'ZData',heli_boom_trans(:,3)-POS_UAV_Z(idx_UAV));
            set(tail(idx_UAV),'Xdata',heli_tail_trans(:,1)+POS_UAV_X(idx_UAV),'YData',-heli_tail_trans(:,2)-POS_UAV_Y(idx_UAV),'ZData',heli_tail_trans(:,3)-POS_UAV_Z(idx_UAV));

            if (iter_ANI > 1)
                plot3(POS_UAV(iter_ANI-1:iter_ANI,6*(idx_UAV-1)+1),-POS_UAV(iter_ANI-1:iter_ANI,6*(idx_UAV-1)+2),-POS_UAV(iter_ANI-1:iter_ANI,6*(idx_UAV-1)+3),'color', clr_uav(idx_UAV,1:3));
            end
        end
        drawnow;
    end
    
      axis([-100 100 -100 100 0 50]);
    if (NUM_OBS ~= 1)
        idx_SCAN = 1;
        for iter_SCAN = 1 : length(POS_Scan)
            if (POS_Scan(iter_SCAN,5) == iter_ANI)
                SCAN_plot(idx_SCAN,1:3) = POS_Scan(iter_SCAN,1:3);
                idx_SCAN = idx_SCAN + 1;
            end
        end

        %axis equal;
         view(-44, 42);
        if (idx_SCAN > 1)
            plot3(SCAN_plot(:,1),-SCAN_plot(:,2),-SCAN_plot(:,3),'r.');
        end
        hold on;

        clear SCAN_plot;
    end
    drawnow;
    if (deter_movie_make == 1)
        frame = getframe(gcf);
        aviobj = addframe(aviobj, frame);
    end
    
end

hold off;

if (deter_movie_make == 1)
   aviobj = close(aviobj);
end
