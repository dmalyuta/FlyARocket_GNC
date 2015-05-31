 function [xNext,PNext] = kalmanFnc(x,P,z,Q,R,dt)
    % Real-time Kalman filter
    
    % Discrete time model matrices A,C (x(k+1)=(A+I*dt)*x(k) ; y=C*x)
    A = eye(2)+dt*[0,1;0,0];
    C = [1 0];%y=Cx

    %Prediction
    x = A*x;
    P = A*P*A'+Q;

    %Update
    inn   = z-C*x;
    S     = C*P*C'+R;
    K     = P*C'*inv(S);
    xNext = x + K*inn;
    PNext = (eye(2)-K*C)*P;
end