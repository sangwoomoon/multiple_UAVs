  clc; 
  clear ; 
  close all ; 

load test5_uav3Swarm1

dat1 = dat' ;
dat1(:,1) = dat1(:,1) - dat1(1,1) ; % dat(1,1) : GPS start time

t_s = 1066 ; 
t_f = 1327 ; 


figure() 
plot(dat1(t_s:t_f,51),dat1(t_s:t_f,50)) ; 
grid on; hold on;
plot(dat1(t_s:t_f,45),dat1(t_s:t_f,44),'b*');
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
plot(dat1(t_s:t_f,1),dat1(t_s:t_f,16)+dat1(t_s:t_f,17)-(0.5766+0.5766),'b'); %aileron
grid on; hold on;
plot(dat1(t_s:t_f,1),dat1(t_s:t_f,16)-dat1(t_s:t_f,17)-(0.5766-0.5766),'r'); %elevator
grid on; hold on;
legend('Aileron','Elevator');
xlabel('time [s]');
ylabel('Control Input');


%%
load test5_uav3Swarm2

dat2 = dat' ;
dat2(:,1) = dat2(:,1) - dat2(1,1) ; % dat(1,1) : GPS start time

t_s = 1072 ; 
t_f = 1344 ; 


figure() 
plot(dat2(t_s:t_f,51),dat2(t_s:t_f,50)) ; 
grid on; hold on;
plot(dat2(t_s:t_f,45),dat2(t_s:t_f,44),'b*');
xlabel('x direction [m]');
ylabel('y direction [m]');
title('position result');
axis equal ;

figure()
subplot(3,1,1) % p,q,r 
a=dat2(:,2:4);
velo2=sqrt(a(:,1).*a(:,1)+a(:,2).*a(:,2)+a(:,3).*a(:,3));
plot(dat2(t_s:t_f,1),velo2(t_s:t_f,1),'b'); 
grid on; hold on;
legend('velocity');
xlabel('time [s]');
ylabel('velocity [m/s]');

subplot(3,1,2); % roll, pitch, yaw response
plot(dat2(t_s:t_f,1),dat2(t_s:t_f,5),'b');  % roll
grid on; hold on;
plot(dat2(t_s:t_f,1),dat2(t_s:t_f,6),'c'); % pitch
plot(dat2(t_s:t_f,1),dat2(t_s:t_f,7),'r'); % yaw
legend('Roll','Pitch','Yaw');
xlabel('time [s]');
ylabel('Attitude [deg]');


subplot(3,1,3); % control output to servo
plot(dat2(t_s:t_f,1),dat2(t_s:t_f,16)+dat2(t_s:t_f,17)-(0.6190+0.5894),'b'); %aileron
grid on; hold on;
plot(dat2(t_s:t_f,1),dat2(t_s:t_f,16)-dat2(t_s:t_f,17)-(0.6190-0.5894),'r'); %elevator
grid on; hold on;
legend('Aileron','Elevator');
xlabel('time [s]');
ylabel('Control Input');


%%
load test5_uav3Swarm3

dat3 = dat' ;
dat3(:,1) = dat3(:,1) - dat3(1,1) ; % dat(1,1) : GPS start time



t_s = 1069 ; 
t_f = 1330 ; 


figure() 
plot(dat3(t_s:t_f,51),dat3(t_s:t_f,50)) ; 
grid on; hold on;
plot(dat3(t_s:t_f,45),dat3(t_s:t_f,44),'b*');
xlabel('x direction [m]');
ylabel('y direction [m]');
title('position result');
axis equal ;

figure()
subplot(3,1,1) % p,q,r 
a=dat3(:,2:4);
velo3=sqrt(a(:,1).*a(:,1)+a(:,2).*a(:,2)+a(:,3).*a(:,3));
plot(dat3(t_s:t_f,1),velo3(t_s:t_f,1),'b'); 
grid on; hold on;
legend('velocity');
xlabel('time [s]');
ylabel('velocity [m/s]');

subplot(3,1,2); % roll, pitch, yaw response
plot(dat3(t_s:t_f,1),dat3(t_s:t_f,5),'b');  % roll
grid on; hold on;
plot(dat3(t_s:t_f,1),dat3(t_s:t_f,6),'c'); % pitch
plot(dat3(t_s:t_f,1),dat3(t_s:t_f,7),'r'); % yaw
legend('Roll','Pitch','Yaw');
xlabel('time [s]');
ylabel('Attitude [deg]');


subplot(3,1,3); % control output to servo
plot(dat3(t_s:t_f,1),dat3(t_s:t_f,16)+dat3(t_s:t_f,17)-(0.5742+0.6434),'b'); %aileron
grid on; hold on;
plot(dat3(t_s:t_f,1),dat3(t_s:t_f,16)-dat3(t_s:t_f,17)-(0.5742-0.6434),'r'); %elevator
grid on; hold on;
legend('Aileron','Elevator');
xlabel('time [s]');
ylabel('Control Input');

figure();
a=10;
plot3(dat1(t_s:a:t_f,51),dat1(t_s:a:t_f,50),-dat1(t_s:a:t_f,52),'r*-') ; 
grid on; hold on;
plot3(dat2(t_s:a:t_f,51),dat2(t_s:a:t_f,50),-dat2(t_s:a:t_f,52),'gv-') ; 
plot3(dat3(t_s:a:t_f,51),dat3(t_s:a:t_f,50),-dat3(t_s:a:t_f,52),'bp-') ; 
% plot(dat(t_s:t_f,45),dat(t_s:t_f,44),'b*');
xlabel('x direction [m]');
ylabel('y direction [m]');
zlabel('z direction [m]');
title('position result');
axis equal ;
