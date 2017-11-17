%%%%%%%%% subscribe cust_imu magnetometer readings and convert to a compass heading %%%%%%%%%%%%
function output = OnlineEKF_main()

    rosshutdown;
    clear;
    rosinit;
    close all;

    %clc

    % read magneto readings
    disp('Searching for Active /cust_imu0 Publisher...');

    while ( ~any(strcmp(rostopic('list'),'/cust_imu0')) )
        pause(1)
    end
    
    global loopcount;
    loopcount = 0;
    subMagneto = rossubscriber('/cust_imu0',@subCallbackMagneto);
    subCamInfo = rossubscriber('/cam0/camera_info');


    % create figure
    f1 = figure('Name','Magnetometer Data','pos',[150 150 9000 9000]);
    fs1 = subplot(2,2,1);
    fs2 = subplot(2,2,2);
    fs3 = subplot(2,2,[3 4]);
    
    
    disp('Waiting For Valid Mesages...');
    global sensorReading;
    global EKFstate;
    
    while(loopcount < 1)
        pause(0.1);
    end
    last = 0;
    
    disp('Runinng...');
    while(1)
        if (last == loopcount)
            continue;
        end
        last= loopcount;
        [xraw,yraw] = pol2cart(sensorReading(last).theta, 1);
        [xekf,yekf] = pol2cart(EKFstate(last).theta, 1);
        
        compass(fs1,xraw,yraw);
        title(fs1,'Raw');
        view(fs1,[90 -90]);
        
        pause(0.02);
        
        compass(fs2,xekf,yekf);
        title(fs2,'EKF Filtered');
        view(fs2,[90 -90]);
        
        pause(0.02);
        
        time_grid1 = fix(EKFstate(last).stamp/10)*10;
        
        
        index1 = find([EKFstate.stamp]>=time_grid1,1);
        t_axis=[EKFstate(index1:last).stamp];
        y1_axis=[sensorReading(index1:last).theta];
        y2_axis=[EKFstate(index1:last).theta];
        

        plot(fs3,t_axis,y1_axis*180/pi,t_axis,y2_axis*180/pi);
        
        xlim(fs3,[time_grid1 time_grid1+10 ]);
        
        height = 20;
        mid = EKFstate(last).theta*180/pi;
        low = fix(mid/height);
        ylim( fs3,[ (low*height - height*0.2)  (low*height+height*1.2)]);
        
        pause(0.02);
        
        
    end
end

function subCallbackMagneto (source,poseMagData)

    global loopcount;
    global sensorReading
    global EKFstate
    
    loopcount = loopcount+1;
    
    if (loopcount == 1)
        tic
    end
    
    toc;
    % calibration matrix
    calib_U = [2.3291    0.2930    0.0387;
             0.0    5.5205   -0.7942;
             0.0         0.0    2.4020;];

    calib_c = [-0.2653; 0.1058; 0.0843];
    
    
    
    magrx = poseMagData.Magnetometer.X;
    magry = poseMagData.Magnetometer.Y;
    magrz = poseMagData.Magnetometer.Z;
    
    
    calibData = calib_U*([magrx; magry; magrz]-calib_c);
    magx = calibData(1);
    %magy = calibData(2);
    magz = calibData(3);

    sensorReading(loopcount).stamp = poseMagData.Header.Stamp.Sec + poseMagData.Header.Stamp.Nsec*1e-9;
    sensorReading(loopcount).theta = dir(magx,magz);

    if (loopcount == 1) % initialise EKF state
        EKFstate(1).stamp = 0;
        EKFstate(1).theta = sensorReading(1).theta ;
        EKFstate(1).omega = 0;
        EKFstate(1).cov = [0.1, 0 ; 0 , 0.1]; % initial covariance
        EKFstate(1).delta = 0.1; % arbitrary, not used
        EKFstate(1).K = zeros(2,1); % arbitrary, not used
        return;
    end

    EKFstate(loopcount).stamp = sensorReading(loopcount).stamp - sensorReading(1).stamp;
    EKFstate(loopcount).delta =  EKFstate(loopcount).stamp - EKFstate(loopcount-1).stamp;

    % X = [ direction (rad) ; rotation (rad/sec) ]
    % P = [ cov( direction ) 0 ; 0 cov (rotation) ]
    % U = 0
    % Z = direction (rad)
    % t_delta (rad/sec)

    X = [ EKFstate(loopcount-1).theta ; EKFstate(loopcount-1).omega ];
    P = EKFstate(loopcount-1).cov;
    U = 0;
    Z = sensorReading(loopcount).theta;
    t_delta = EKFstate(loopcount).delta;

    [X_next , P_next , K] = EKFupdate(X,P,U,Z,t_delta);

    EKFstate(loopcount).theta = X_next(1,1);
    EKFstate(loopcount).omega = X_next(2,1);
    EKFstate(loopcount).cov = P_next;
    EKFstate(loopcount).K = K;
    
    
%     persistent callInterval;
%     if isempty(callInterval)
%         callInterval= cputime;
%     end 
%     disp((cputime - callInterval)*1000);  
%     callInterval = cputime;

    tic
end

function output = dir(x,z)

    
    output = zeros(size(x,1),1);
    
    
    for i = 1:size(x,1)
        Azimuth = 0.0;
        magx= x(i);
        magz= z(i);
        
        if (magz == 0 && magx < 0)
            Azimuth = pi*0.5;
        end

        if (magz == 0 && magx > 0)
            Azimuth = pi*1.5;
        end

        if (magz < 0)
            Azimuth = pi - atan(magx/magz);
        end

        if (magz > 0 && magx < 0)
            Azimuth = - atan(magx/magz);
        end

        if (magz > 0 && magx > 0)
            Azimuth = 2*pi - atan(magx/magz);
        end
        
        output(i) = Azimuth;
    end

end
