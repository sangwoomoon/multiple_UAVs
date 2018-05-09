

load test2_uav2Swarm2

dat1 = dat' ;
dat1(:,1) = dat1(:,1) - dat1(1,1) ; % dat(1,1) : GPS start time

t_s = 1; 
t_f = 2880 ; 

plot(dat1(t_s:t_f,51),dat1(t_s:t_f,50)) ; 
grid on; hold on;
plot(dat1(t_s:t_f,45),dat1(t_s:t_f,44),'b*'); % waypoint
xlabel('x direction [m]');
ylabel('y direction [m]');
title('position result');
axis equal ;

figure()
subplot(3,1,1) % p,q,r 
a=dat1(:,2:4);
velo1=sqrt(a(:,1).*a(:,1)+a(:,2).*a(:,2)+a(:,3).*a(:,3));
plot(dat1(t_s:t_f,1),velo1(t_s:t_f,1),'b'); 
grid on; hold on;
legend('velocity');
xlabel('time [s]');
ylabel('velocity [m/s]');

subplot(3,1,2); % roll, pitch, yaw response
plot(dat1(t_s:t_f,1),dat1(t_s:t_f,5),'b');  % roll
grid on; hold on;
plot(dat1(t_s:t_f,1),dat1(t_s:t_f,6),'c'); % pitch
plot(dat1(t_s:t_f,1),dat1(t_s:t_f,7),'r'); % yaw
legend('Roll','Pitch','Yaw');
xlabel('time [s]');
ylabel('Attitude [deg]');


subplot(3,1,3); % control output to servo
plot(dat1(t_s:t_f,1),dat1(t_s:t_f,16)+dat1(t_s:t_f,17)-(0.6176+0.5880),'b'); %aileron
grid on; hold on;
plot(dat1(t_s:t_f,1),dat1(t_s:t_f,16)-dat1(t_s:t_f,17)-(0.6176-0.5880),'r'); %elevator
grid on; hold on;
legend('Aileron','Elevator');
xlabel('time [s]');
ylabel('Control Input');

