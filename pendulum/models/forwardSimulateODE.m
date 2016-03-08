function [t,states,controls] = forwardSimulateODE(stateInit,controller,dt,nSteps,physicalParams)
    %FORWARDSIMULATEODE
    %
    % [t,states,controls] = FORWARDSIMULATEODE(stateInit,controller,dt,nSteps,physicalParams)
    %
    % stateInit      - [1,2] array.
    % controller       - Function handle
    % dt             - Scalar timestep.
    % physicalParams - Struct with fields ('m','l','g')
    %
    % states         - [nSteps,2] array.
    % controls       - [nSteps,1] array of torques.

    % unpack physical params
    if isfield(physicalParams,'m')
        m = physicalParams.m;
    else
        error('m not a field in physicalParams.');
    end
    if isfield(physicalParams,'l')
        l = physicalParams.l;
    else
        error('l not a field in physicalParams.');
    end
    if isfield(physicalParams,'g')
        g = physicalParams.g;
    else
        g = 9.81;
    end
    
    % states(i,1) = theta
    % states(i,2) = theta dot
    states = zeros(nSteps,2);
    state = stateInit;
    
    % motion noise
    if isfield(physicalParams,'controlNoise')
        controlNoise = randn(nSteps,1)*sqrt(physicalParams.controlNoise.variance);
    else
        controlNoise = zeros(nSteps,2);
    end
    
    controls = zeros(nSteps,1);
    t = [1:nSteps]*dt;
    
    for i = 1:nSteps
        controls(i) = controller(t(i)-dt,state)+controlNoise(i);
        derivatives = @(t,x) systemDerivatives(t,x,controls(i),physicalParams);
        [~,odeOut] = ode45(derivatives,[t(i)-dt t(i)],state);
        state = odeOut(end,:);
        % wrap angle to [0,2*pi] 
        % otherwise unexpected things might happen
        state(1) = mod(state(1),2*pi);
        states(i,:) = state;
    end
end