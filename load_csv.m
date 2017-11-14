%%% Preload data from csv

if isunix
    filePath = '/home/dhl/Desktop/mag/14-26-44.csv';
elseif ispc 
    filePath = 'D:\TLAB_Work\14-26-44.csv';
else
    disp('Not Supported');
end

%import csv file with header row
bag = readtable(filePath);
fieldnames(bag)


T_raw = bag.field_header_stamp;
Mx_raw = bag.field_magnetometer_x;
My_raw = bag.field_magnetometer_y;
Mz_raw = bag.field_magnetometer_z;

%Zeroing Start Time, and remove deplication (multiples of 4)
% calibration matrix
U = [2.3291    0.2930    0.0387;
         0    5.5205   -0.7942;
         0         0    2.4020;];
     
c = [-0.2653; 0.1058; 0.0843];

initial_time = T_raw(1,1);
new_size = fix (size(T_raw,1)/4);

for index = 1:new_size
    rawData = [ Mx_raw(index*4-3) ; My_raw(index*4-3) ; Mz_raw(index*4-3) ]  ;
    calibData = U*(rawData-c);
    preProcssedData (index, :) =  [ (T_raw(index*4-3,1) - initial_time )/1.0e9, calibData.'];
end

T = preProcssedData(:,1);
Mx = preProcssedData(:,2);
My = preProcssedData(:,3);
Mz = preProcssedData(:,4);

%plot(T,Mx,T,Mz);

sys0 = azimuth(Mx,Mz);
sys = [ T , azimuth(Mx,Mz) ];


rN = 0;

for i = 2:size(sys,1)
    if (sys0(i) - sys0(i-1) < -pi) % current value fold over
        rN = rN + 1;
    elseif (sys0(i) - sys0(i-1) > pi) % current value fold back
        rN = rN - 1;
    end
    sys(i,2) = sys0(i) + rN*2*pi;
end

