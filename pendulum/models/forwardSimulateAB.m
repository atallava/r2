function [t,states] = forwardSimulateAB(stateInit,controls,dt,A,B,affineTerm)
    %FORWARDSIMULATE
    %
    % states = FORWARDSIMULATE(stateInit,controls,dt,physicalParams)
    %
    % stateInit      - [1,2] array. 
    % controls       - [nSteps,1] array of torques.
    % dt             - Scalar timestep.
    % physicalParams - Struct with fields ('m','l','g')
    %
    % states         - [nSteps,2] array. 
    
    if nargin < 5
        affineTerm = zeros(size(stateInit));
    end

    % states(i,1) = theta
    % states(i,2) = theta dot
    nSteps = size(controls,1);
    states = zeros(nSteps,2);
    state = stateInit;
    t = [1:nSteps]*dt;
    
    for i = 1:nSteps
        % A, B require state to be column vector
        stateNext = A*state'+B*controls(i)';
        states(i,:) = stateNext;
        state = stateNext;
    end
    states = bsxfun(@plus,states,affineTerm);
end