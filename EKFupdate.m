% X = [ direction (rad) ; rotation (rad/sec) ]
% P = [ cov( direction ) 0 ; 0 cov (rotation) ]
% U = 0
% Z = direction (rad)
% t_delta (rad/sec)


function [X_next , P_next , K] = EKFupdate(X,P,U,Z,t_delta)

    % X_hat = F*X + B*U    the predicted state, based on previous state and
    % input
    F = [1  t_delta ; 0  1 ];
    B = [ 0 ; t_delta];
    X_hat =  F*X + B*U;
    X_hat(1) = mod ( X_hat(1), 2*pi);
    
    
    % P_hat = G*P*G.' + Q
    % G is local-linearised F
    % for this model, F is actually linear
    Q = [ 1e-6 , 0 ; 0 , 3e-4];
    P_hat = F*P*(F.') + Q
    
    % H is the transformation from sensor reading to state
    % Z_expected = H*X_hat
    
    H = [ 1 0 ]; % direct feeding of theta to expected Z
    
    % Kalman Gain
    R = 1e-2;
    K = P_hat*(H.')*inv(H*P_hat*(H.') + R);
    
    Z_expected = H*X_hat;
    % if Z jumps at boundary condition at 0 or 360 degree
    if ( abs(Z - Z_expected) > pi/2)
       
        if (Z > Z_expected)
            Z = Z - 2*pi;
        else
            Z = Z + 2*pi;
        end
        
    end
     
    X_next = X_hat + K*( Z - Z_expected );
    X_next (1) = mod (X_next (1),2*pi);
    
    P_next = P_hat - K*H*P_hat;
    

end