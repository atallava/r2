function [tVec,states,controls,statesDot] = forwardSimulateODEWithController(stateInit,controller,dt,nSteps,physicalParams)
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
    
    derivatives = @(t,x) systemDerivativesWithController(t,x,controller,physicalParams);
    tVec = 0:dt:(nSteps*dt);
    [~,states] = ode45(derivatives,tVec,stateInit);
    controls = controller(tVec,states);
    
    % system derivatives need column states
    statesDot = systemDerivativesWithControls(tVec,states',controls,physicalParams);
    statesDot = statesDot';
end
