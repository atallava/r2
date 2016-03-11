function predictorState = transformDynStateToGpState(dynState)
    %TRANSFORMDYNSTATETOGPSTATE 
    %
    % predictorState = TRANSFORMDYNSTATETOGPSTATE(dynState)
    %
    % dynState       - [nStates,2] array.
    %
    % predictorState - [nStates,1] array.
    
    % picks up only theta
    predictorState = dynState(:,1);
end