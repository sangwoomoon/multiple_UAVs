function [P, V, P_f] = Set_UAV(initial_velocity,initial_height,final_height,iter_UAV)

figure(1)
grid on
hold on
axis equal tight
axis([-100 100 -100 100 0 50]);

Mat2NED_xyz = [ 1 0 0; 0 -1 0; 0 0 -1];
Mat2NED_all = [Mat2NED_xyz zeros(3,3); zeros(3,3) Mat2NED_xyz];

[P_0xy(1),P_0xy(2)] = ginput(1);  % matlab coordinate
P = (Mat2NED_all*[P_0xy(1) P_0xy(2) initial_height 0 0 0]')';    %  NED coordinate
plot3(P(1),-P(2),-P(3),'bo');  % matlab coordinate
text(P(1),-P(2),-P(3), sprintf('%d',iter_UAV));  % matlab coordinate

[P_fxy(1),P_fxy(2)] = ginput(1);  % matlab coordinate
P_f = (Mat2NED_all*[P_fxy(1) P_fxy(2) final_height 0 0 0]')';    %  NED coordinate
plot3(P_f(1),-P_f(2),-P_f(3),'ro');  % matlab coordinate

[P_vxy(1),P_vxy(2)] = ginput(1);  % matlab coordinate
vector = [P(1),-P(2),-P(3);P_vxy(1),P_vxy(2),-P(3)];   % matlab coordinate
plot3(vector(:,1),vector(:,2),vector(:,3));   % matlab coordinate

% velocity assign (from scalar to vector)
V = (Mat2NED_all*(initial_velocity*[(P_vxy(1)-P_0xy(1)) (P_vxy(2)-P_0xy(2)) 0 0 0 0]/norm([(P_vxy(1)-P_0xy(1)) (P_vxy(2)-P_0xy(2)) 0]))')';
P(6) = atan2(-P_vxy(2)+P_0xy(2),P_vxy(1)-P_0xy(1));   % NED coordinate

