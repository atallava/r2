function xDot = systemDerivativesWithPredictor(t,x,predictorStruct)
    %SYSTEMDERIVATIVESWITHPREDICTOR
    %
    % xDot = SYSTEMDERIVATIVESWITHPREDICTOR(t,x,predictor,dynStateToPredictorState)
    %
    % t                        - nX length vector. Times.
    % x                        - [2,nX] array. States.
    % predictorStruct          - Struct with fields
    % ('predictor','dynStateToPredictorState').
    %
    % xDot                     - [2,nX] array. Velocities.

    if isfield(predictorStruct,'predictor')
        predictor = predictorStruct.predictor;
    else
        error('predictor must be a field of predictorStruct.');
    end
    
    if isfield(predictorStruct,'dynStateToPredictorState')
        dynStateToPredictorState = predictorStruct.dynStateToPredictorState;
    else
        error('dynStateToPredictorState must be a field of predictorStruct.');
    end
    
    nX = size(x,2);
    condn = length(t) == nX;
    assert(condn,'Number of time steps should equal number of states. Failed check.');
    
    xDot = zeros(size(x));
    xDot(1,:) = x(2,:);
    
    % transform works with row states
    predictorStates = dynStateToPredictorState(x');
    accns = predictor(predictorStates);
    xDot(2,:) = accns';
end