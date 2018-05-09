p_task=p_f;

D2R = pi/180;
Init_ref_lat     =   37.46800994873*D2R;     % [rad]
Init_ref_long    =   126.44860839844*D2R;     % [rad]
Init_ref_h       =   25.48;                     % [ft]


[X Y Z]=ned2xyz(Init_ref_lat, Init_ref_long,Init_ref_h, p_task(:,2)*50, p_task(:,1)*50,25.48*ones(size(p_task,1),1))

[lat long h]=xyz2llh(X,Y,Z)

lat = lat*180/pi
long = long*180/pi

lat_min = (lat-double(int16(lat)))*60
long_min = (long-double(int16(long)))*60