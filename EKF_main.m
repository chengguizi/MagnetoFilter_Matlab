
close all

if isunix
    filePath = '/home/dhl/Desktop/mag/2017-10-02-14-26-44.bag';
elseif ispc 
    filePath = 'D:\TLAB_Work\2017-09-20-10-13-12.bag';
else
    disp('Not Supported');
end

if ~exist('matx')
    matx = loadRosBag(filePath);
end


disp('Plotting...');

figure
T = matx(:,1);
Mx = matx(:,2);
My = matx(:,3);
Mz = matx(:,4);
plot(T,Mx,T,Mz);


figure
plot(T, (Mx.^2 + My.^2 + Mz.^2).^0.5);


mat_azimuth = [ matx(:,1) , dir(matx(:,2),matx(:,4)) ];
    



%initialise EKF

% X = [ direction (rad) ; rotation (rad/sec) ]
% P = [ cov( direction ) 0 ; 0 cov (rotation) ]
% U = 0
% Z = direction (rad)
% t_delta (rad/sec)

EKF_azimuth = { mat_azimuth(1,2) , [ mat_azimuth(1,2) ; 0] ,  [0.1, 0 ; 0 , 0.1] , zeros(2,1)}
U=0;

for step = 2: size (mat_azimuth,1)
    X = EKF_azimuth{step-1, 2};
    P = EKF_azimuth{step-1, 3};
    LP = EKF_azimuth{step-1,1}*0.95 + mat_azimuth(step,2)*0.05;
    t_delta = mat_azimuth(step,1) - mat_azimuth(step-1,1);
    
    
    [X_next , P_next , K] = EKFupdate(X,P,U,mat_azimuth(step,2),t_delta);
    
    EKF_azimuth (end+1, :) = { LP , X_next , P_next , K};
end


figure
T=mat_azimuth(:,1);
LP_result = cell2mat(EKF_azimuth(:,1));
A_raw=mat_azimuth(:,2);

A_EKF = cell2mat(EKF_azimuth(:,2));
A_EKF = A_EKF (1:2:end,:);

plot(T,A_raw,T,A_EKF,T,LP_result);

A_K = cell2mat(EKF_azimuth(:,4));
A_K1 = A_K(1:2:end,:);
A_K2 = A_K(2:2:end,:);


figure
plot(T,A_K1,T,A_K2);
axis ([ 1 65 0 2]);



function output = dir(x,z)


    % Azimuth = atan2(Yh, Xh);
    % magx = Yh;
    % magz = Xh;
    
    output = zeros(size(x,1),1);
    
    for i = 1:size(x,1)
        
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