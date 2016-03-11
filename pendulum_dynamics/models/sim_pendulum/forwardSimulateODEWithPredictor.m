function [tVec,states,statesDot] = forwardSimulateODEWithPredictor(stateInit,dt,nSteps,predictorStruct)
    %FORWARDSIMULATEODEWITHCONTROLLER
    %
    % [tVec,states,controls,statesDot] = FORWARDSIMULATEODEWITHCONTROLLER(stateInit,controller,dt,nSteps,physicalParams)
    %
    % stateInit      - [1,2] array.
    % controller     - Function handle.
    % dt             - Scalar. Timestep.
    % nSteps         - Scalar. Number of steps to forward simulate for.
    % physicalParams - Struct with fields ('m','l','g').
    %
    % tVec           - [nSteps+1,1] array.
    % states         - [nSteps+1,2] array. Includes stateInit.
    % controls       - [nSteps+1,1] array. Controls at tVec.
    % statesDot      - [nSteps+1,2] array. Velocities at tVec.
    
    if ~isfield(predictorStruct,'predictor')
        error('predictor must be a field of predictorStruct.');
    end
    
    if ~isfield(predictorStruct,'dynStateToPredictorState')
        error('dynStateToPredictorState must be a field of predictorStruct.');
    end

    derivatives = @(t,x) systemDerivativesWithPredictor(t,x,predictorStruct);
    tVec = 0:dt:(nSteps*dt);
    [~,states] = ode45(derivatives,tVec,stateInit);
    
    % special case. nSteps = 1, or, tVec = [t0 tf]
    % ode45 returns a bunch of intermediate states
    if length(tVec) == 2
        states = states([1 end],:);
    end
        
    % system derivatives need column states
    statesDot = systemDerivativesWithPredictor(tVec,states',predictorStruct);
    statesDot = statesDot';
end
