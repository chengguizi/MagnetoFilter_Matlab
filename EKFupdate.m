% X = state vector
% P = covariance, updated by EKF
% Q = state noise (external uncertainties)
% R = measurement noise
% U = input vector
% Z = measurement vector
% F = state update matrix
% B = input conversion matrix
% H = state to measurement conversion matrix

function [X_next , P_next , K] = EKFupdate(X,P,Q,R,U,Z,F,B,H)

    % X_hat = F*X + B*U    the predicted state, based on previous state and
    % input
    X_hat =  F*X + B*U;
    
    % P_hat = G*P*G.' + Q
    % G is local-linearised F
    % for this model, F is actually linear
    P_hat = F*P*(F') + Q;
    
    % Kalman Gain
    K = P_hat*(H')*inv(H*P_hat*(H') + R);
    
    % H is the transformation from sensor reading to state
    Z_expected = H*X_hat;
     
    X_next = X_hat + K*( Z - Z_expected );
    
    P_next = P_hat - K*H*P_hat;

end