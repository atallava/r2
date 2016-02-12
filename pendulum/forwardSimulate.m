function states = forwardSimulate(stateInit,controls,dt,physicalParams)
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

    nSteps = size(controls,1);
    states = zeros(nSteps,2);
    state = stateInit;
    % euler integration
    for i = 1:nSteps
        accn = (controls(i)-m*g*l*sin(state(1)))/...
            (m*l^2);
        state(1) = state(1)+state(2)*dt;
        state(2) = state(2)+accn*dt;
        states(i,:) = state;
    end
end