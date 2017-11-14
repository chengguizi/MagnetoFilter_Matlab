
clear; close all;clc;

load_csv;
%plot(sys(:,1),sys(:,2));

matx = sys;
disp('Plotting...');


%initialise EKF

% X = [ direction (rad) ; rotation (rad/sec) ]
% P = [ cov( direction ) 0 ; 0 cov (rotation) ]
% U = 0
% Z = direction (rad)
% t_delta (rad/sec)

%%% EKF_azimuth row = [ expotential filter, EKF state, EKF covariance, EKF Gain K]
EKF_azimuth = { sys(1,2) , [ sys(1,2) ; 0] ,  [0.1, 0 ; 0 , 0.1] , zeros(2,1)};

EKF_init

for step = 2: size (sys,1)
    X = EKF_azimuth{step-1, 2};
    P = EKF_azimuth{step-1, 3};
    
    LP1 = EKF_azimuth{step-1,1};
    LP2 = sys(step,2); % new measurement
    
    LP = LP1*LPk + LP2*(1-LPk);
    
    [X_next , P_next , K] = EKFupdate(X,P,Q,R,U,sys(step,2),t_delta);
    
    EKF_azimuth (end+1, :) = { LP , X_next , P_next , K};
end


figure(1)
%T=mat_azimuth(:,1);
LP_result = cell2mat(EKF_azimuth(:,1));
A_raw=sys(:,2);

A_EKF = cell2mat(EKF_azimuth(:,2));
A_EKF1 = A_EKF (1:2:end,:);
A_EKF2 = A_EKF (2:2:end,:);

plot(A_raw);
hold on
plot(A_EKF1);
plot(LP_result,'--');
legend('Measurement','EKF Est.','Expo Filter Est.');

A_K = cell2mat(EKF_azimuth(:,4));
A_K1 = A_K(1:2:end,:);
A_K2 = A_K(2:2:end,:);


figure(2)
plot(A_K1);
hold on
plot(A_K2);

figure(3)
plot(A_EKF2);


%%% save as CSV %%%

filename = [num2str(yyyymmdd(datetime)) , '-ekf.csv'];
csvwrite(filename,[A_EKF1,A_EKF2]);

filename2 = [num2str(yyyymmdd(datetime)) , '-data.csv'];
csvwrite(filename2,A_raw);
