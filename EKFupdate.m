% X = [ direction (rad) ; rotation (rad/sec) ]
% P = [ cov( direction ) 0 ; 0 cov (rotation) ]
% U = 0
% Z = direction (rad)
% t_delta (rad/sec)


function [X_next , P_next] = EKFupdate(X,P,U,Z,t_delta)

    % X_hat = F*X + B*U    the predicted state, based on previous state and
    % input
    F = [1  t_delta ; 0  1 ];
    B = [ 0 ; t_delta];
    X_hat =  F*X + B*U;
    
    
    % P_hat = G*P*G.' + Q
    % G is local-linearised F
    % for this model, F is actually linear
    Q = 0.001*eye(2);
    P_hat = F*P*(F.') + Q;
    
    % H is the transformation from sensor reading to state
    % Z_expected = H*X_hat
    
    H = [ 1 0 ]; % direct feeding of theta to expected Z
    
    % Karman Gain
    R = 0.05;
    K = P_hat*(H.')*inv(H*P_hat*(H.') + R);
    
    
    X_next = X_hat + K*( Z - H*X_hat );
    X_next (1) = mod (X_next (1),2*pi);
    
    P_next = P_hat - K*H*P_hat;
    

end