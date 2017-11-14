t_delta = 0.02; %sys(step,1) - sys(step-1,1);

cov_theta = (t_delta * 0.08)^2;
cov_omega = (t_delta * pi/2)^2;
Q = [ cov_theta , 0 ; 0 , cov_omega]; % External uncertainties
R = 0.8e-3; % sensor noise
U=0;



%%%%% Low Pass filter%%%%

LPk = 0.95;