%%%%%%%%% subscribe cust_imu magnetometer readings and convert to a compass heading %%%%%%%%%%%%

rosshutdown;
setenv('ROS_HOSTNAME','172.17.21.12');
setenv('ROS_IP','172.17.21.12');
setenv('ROS_MASTER_URI','http://172.16.143.85');
rosinit('172.16.143.85');
% rosinit('192.168.154.131', 'NodeHost','192.168.1.1','NodeName','/test_node')
close all;
%clear;
%clc;



% read pose and vicon topics
scanMagne = rossubscriber('/cust_imu0');
test = rossubscriber('/cam0/camera_info');
rostopic info cust_imu0
%rostopic echo cam0/camera_info
%pause

% calibration matrix
U = [2.3291    0.2930    0.0387;
         0    5.5205   -0.7942;
         0         0    2.4020;];
     
c = [-0.2653; 0.1058; 0.0843];

% create figure
figure;

PI     = 3.1415926535897;
Azimuth = 0.0;

% loop to display
while(1)
    
    % receive vision pose messages
    poseMagData = receive(scanMagne);
%     accx = poseMagData.LinearAcceleration.X;
%     accy = poseMagData.LinearAcceleration.Y + 9.81;
%     accz = poseMagData.LinearAcceleration.Z;
    
    magrx = poseMagData.Magnetometer.X;
    magry = poseMagData.Magnetometer.Y;
    magrz = poseMagData.Magnetometer.Z;
    
    rawData = [magrx; magry; magrz];
    
    calibData = U*(rawData-c);
    magx = calibData(1);
    magy = calibData(2);
    magz = calibData(3);
    
    
%     pitch = 180 * atan2(accz, sqrt(accx*accx + accy*accy)) / PI;
%     roll = 180 * atan2(accx, accy) / PI;
    
    % Xh = magz*cos(pitch) + magx*sin(roll)*sin(pitch) - magy*cos(roll)*sin(pitch);
    % Yh = magx*cos(roll) + magy*sin(roll);
    
    % Azimuth = atan2(Yh, Xh);
    % magx = Yh;
    % magz = Xh;
    if (magz == 0 && magx < 0)
        Azimuth = 90.0;
    end
    
    if (magz == 0 && magx > 0)
        Azimuth = 270.0;
    end
    
    if (magz < 0)
        Azimuth = 180 - atan(magx/magz)*180/PI;
    end
    
    if (magz > 0 && magx < 0)
        Azimuth = - atan(magx/magz)*180/PI;
    end
    
    if (magz > 0 && magx > 0)
        Azimuth = 360 - atan(magx/magz)*180/PI;
    end
    
    [x,y] = pol2cart(Azimuth/180.0*PI, 1);
    compass(x,y)
    view([90 -90])
    
    pause(0); refresh;
    
end

