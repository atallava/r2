function xDot = systemDerivatives(t,x,controls,physicalParams)
    %SYSTEMDERIVATIVES
    %
    % xDot = SYSTEMDERIVATIVES(t,x,controls,physicalParams)
    %
    % t              - Scalar.
    % x              - Column vector.
    % controls       -
    % physicalParams -
    %
    % xDot           - Column vector.

    xDot(1) = x(2);
    xDot(2) = (controls-physicalParams.m*physicalParams.g*physicalParams.l*sin(x(1)))/...
        (physicalParams.m*physicalParams.l^2);
    
    % columnification
    xDot = xDot';
end