

filePath = '/home/dhl/Desktop/mag/2017-10-02-14-26-44.bag';

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

EKF_azimuth = { mat_azimuth(1,1) , [ mat_azimuth(1,2) ; 0] ,  [1, 0 ; 0 , 1] }
U=0;

for step = 2: size (mat_azimuth,1)
    X = EKF_azimuth{step-1, 2};
    P = EKF_azimuth{step-1, 3};
    t_delta = mat_azimuth(step,1) - mat_azimuth(step-1,1);
    
    
    [X_next , P_next] = EKFupdate(X,P,U,mat_azimuth(step,2),t_delta);
    
    EKF_azimuth (end+1, :) = { mat_azimuth(step,1) , X_next , P_next};
end


figure
T=mat_azimuth(:,1);
A_raw=mat_azimuth(:,2);

A_EKF = cell2mat(EKF_azimuth(:,2));
A_EKF = A_EKF (1:2:end,:)
plot(T,A_raw,T,A_EKF)



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