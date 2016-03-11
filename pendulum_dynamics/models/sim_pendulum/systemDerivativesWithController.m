function xDot = systemDerivativesWithController(t,x,controller,physicalParams)
    %SYSTEMDERIVATIVESWITHCONTROLLER
    %
    % xDot = SYSTEMDERIVATIVESWITHCONTROLLER(t,x,controls,physicalParams)
    %
    % t              - nStates length vector.
    % x              - [2,nStates] array.
    % controller       - Function handle.
    % physicalParams - Struct.
    %
    % xDot           - [2,nStates] array.
    
    % controller assumes states as [nStates,2]
    controls = controller(t,x'); % [nStates,1]
    
    xDot = systemDerivativesWithControls(t,x,controls,physicalParams);
end