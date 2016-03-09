function [A,B,ABResidual] = linearizeAboutDesired(stateDesired,controlDesired,dt,physicalParams)
    %LINEARIZEABOUTDESIRED 
    % NOTE: A,B calculated with state represented as vector.
    %
    % [A,B,ABResidual] = LINEARIZEABOUTDESIRED(stateDesired,controlDesired,dt,physicalParams)
    %
    % stateDesired   - [1,2] vector.
    % controlDesired - Scalar.
    % dt             - Scalar. 
    % physicalParams - Struct.
    %
    % A              - [2,2] array.
    % B              - [2,1] array.
    % ABResidual     - [1,2] vector.
    
    % derivative wrt state
    dFdX = @(x) ...
        [0 1; ...
        -physicalParams.g/physicalParams.l*cos(x(1)) 0];
    dFdU = @(x) ...
        [0; ....
        1/(physicalParams.m*physicalParams.l^2)];
    
    % linearization
    A = eye(2,2) + ...
        dFdX(stateDesired)*dt;
    
    B = dFdU(stateDesired)*dt;
    
    % state residual
    [~,stateNextOde] = forwardSimulateODE(stateDesired,constantController(controlDesired),...
        dt,1,physicalParams);
    stateNextAB = A*stateDesired'+B*controlDesired;
    ABResidual = stateNextOde-stateNextAB';
end