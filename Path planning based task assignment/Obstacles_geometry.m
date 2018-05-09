clc;
clear all; 
close all;

load Obstacles_LLH.mat

Obstacles_LLH(:,2:3) = Obstacles_LLH(:,2:3)*1e-4 ;

D2R = pi/180;


Num_pt = size(Obstacles_LLH,1);

for i= 1:Num_pt 
    
    lat = Obstacles_LLH(i,2);
    long = Obstacles_LLH(i,3);
    h = 0 ; 
    [XYZ(i,1), XYZ(i,2), XYZ(i,3)] = llh2xyz(lat*D2R,long*D2R,h); 
    
      

end
XYZ_ref = XYZ(1,:);
% figure; plot3( XYZ(:,1), XYZ(:,2), XYZ(:,3),'*'); hold on;
% figure; plot(( XYZ(:,1)-XYZ_ref(1)), (XYZ(:,2)-XYZ_ref(2)),'*'); hold on;



for i = 1:Num_pt
    
    [NED(i,1), NED(i,2), NED(i,3)]=xyz2ned(XYZ_ref(1),XYZ_ref(2),XYZ_ref(3),XYZ(i,1), XYZ(i,2), XYZ(i,3));
        
end

% figure; plot(NED(:,2),NED(:,1),'+');

obstacle_oem = [NED(:,2) NED(:,1)] * 2e-2 ;
obstacle_oem(:,2) = obstacle_oem(:,2)-80 ; 
obstacle_oem(:,1) = obstacle_oem(:,1)+40 ; 

% plot(obstacle_oem(:,1),obstacle_oem(:,2),'*');

obstacle_input(1:3,1:2) = obstacle_oem(2:4,:);
obstacle_input(4,1:2) = obstacle_oem(8,:);
obstacle_input(5:8,1:2) = obstacle_oem(4:7,:);
obstacle_input(9:12,1:2) = obstacle_oem(9:12,:);
obstacle_input(13:16,1:2) = obstacle_oem(13:16,:);
obstacle_input(17:20,1:2) = obstacle_oem(17:20,:);


vrt(1:3,1:2) = obstacle_oem(2:4,:);
vrt(4,1:2) = obstacle_oem(8,:);
vrt(5,1:2) = obstacle_oem(2,:);
vrt(1:5,3) = 1 ; 

vrt(6:9,1:2) = obstacle_oem(4:7,:);
vrt(10,1:2) = obstacle_oem(4,:);
vrt(6:10,3) = 2 ; 

vrt(11:14,1:2) = obstacle_oem(9:12,:);
vrt(15,1:2) = obstacle_oem(9,:);
vrt(11:15,3) = 3 ; 

vrt(16:19,1:2) = obstacle_oem(13:16,:);
vrt(20,1:2) = obstacle_oem(13,:);
vrt(16:20,3) = 4 ; 

vrt(21:24,1:2) = obstacle_oem(17:20,:);
vrt(25,1:2) = obstacle_oem(17,:);
vrt(21:25,3) = 5 ; 

j = 1 ; 
for i = 1: size(vrt,1)
   vrt(i,4) = j ; 
   if j == 5 
       vrt(i,4) = 1 ; 
       j = j - 5 ; 
   else 
       line(vrt(i:i+1,1),vrt(i:i+1,2)); hold on; 
   end
   j = j + 1 ; 
    
end



vrt_config = zeros(size(vrt,1),6);
vrt_config = vrt ;


% p_i = 
% p_f = 
% p_task = 


% % plot(x(1:7,1),x(1:7,2),'*'); hold on; 