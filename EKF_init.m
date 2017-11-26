t_delta = 0.02; %sys(step,1) - sys(step-1,1);

cov_theta = (t_delta * 0.08)^2;
cov_omega = (t_delta * pi/2)^2;
Q = [ 1e-3 , 0 ; 0 , 1e-2]; % External uncertainties
R = 0.1; % sensor noise
U=0;


F = [1  t_delta ; 0  1 ];
B = [ 0 ; t_delta];
H = [ 1 0 ]; % direct feeding of theta to expected Z


%%%%% Low Pass filter%%%%

LPk = 0.95;