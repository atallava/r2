function [t,states,controls] = forwardSimulateWithLinearController(stateInit,K,dt,physicalParams,nSteps)
    %FORWARDSIMULATEWITHLINEARCONTROLLER
    %
    % [states,controls] = FORWARDSIMULATEWITHLINEARCONTROLLER(stateInit,K,dt,physicalParams,nSteps)
    %
    % stateInit      -
    % K              - 
    % dt             -
    % physicalParams -
    % nSteps         -
    %
    % states         -
    % controls       -

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
    controls = zeros(nSteps,1);
    state = stateInit;
    t = [1:nSteps]*dt;
    
    for i = 1:nSteps
        controls(i) = dot(K,state);
        derivatives = @(t,x) systemDerivatives(t,x,controls(i),physicalParams);
        [~,odeOut] = ode45(derivatives,[t(i)-dt t(i)],state);
        state = odeOut(end,:);
        states(i,:) = state;
    end
end