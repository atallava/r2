function xDot = systemDerivativesWithControls(t,x,controls,physicalParams)
    %SYSTEMDERIVATIVESWITHCONTROLS
    %
    % xDot = SYSTEMDERIVATIVESWITHCONTROLS(t,x,control,physicalParams)
    %
    % t              - nX length vector.
    % x              - [2,nX] array.
    % control        - nX length vector.
    % physicalParams - Struct.
    %
    % xDot           - [2,nX] array.
    
    nX = size(x,2);
    condn = length(t) == nX;
    assert(condn,'Number of time steps should equal number of states. Failed check.');
    condn = length(controls) == nX;
    assert(condn,'Number of controls should equal number of states. Failed check.');
        
    xDot = zeros(size(x));
    xDot(1,:) = x(2,:);
    controls = flipVecToRow(controls);
    controlMoment = controls/(physicalParams.m*physicalParams.l^2);
    gravityMoment = -physicalParams.m*physicalParams.g*physicalParams.l*...
        sin(x(1,:))./(physicalParams.m*physicalParams.l^2);
    xDot(2,:) = controlMoment+gravityMoment;
end