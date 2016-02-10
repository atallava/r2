function [states,linVel,latVel,angVel] = predTrajGp(stateInit,controls,dt,gpLinVel,gpLatVel,gpAngVel)\
    %PREDTRAJGP
    %
    % [states,linVel,latVel,angVel] = PREDTRAJGP(stateInit,controls,dt,gpLinVel,gpLatVel,gpAngVel)
    %
    % stateInit   - Length 3 vector.
    % controls    - [nMeasurements,2] array.
    % dt          - nMeasurements length vector.
    % gpLinVel - Function handle.
    % gpAngVel - Function handle.
    % gpLatVel - Function handle.
    %
    % states      - [nMeasurements+1,3] array.
    % linVel      - nMeasurements length vector.
    % latVel      - nMeasurements length vector.
    % angVel      - nMeasurements length vector.
    
    nTimesteps = length(dt);
    dimState = length(stateInit);
    dimControls = size(controls,2);
    states = zeros(nTimesteps+1,dimState);
    states(1,:) = stateInit;
    linVel = zeros(1,nTimesteps);
    latVel = zeros(1,nTimesteps);
    angVel = zeros(1,nTimesteps);
    xQuery = zeros(nTimesteps,dimState+dimControls);
        
    for i = 1:nTimesteps
        xQuery(i,:) = dynamicsVarsToGpStates(states(i,:),controls(i,:));
        linVel(i) = gpLinVel(xQuery(i,:));
        latVel(i) = gpLatVel(xQuery(i,:));
        angVel(i) = gpAngVel(xQuery(i,:));
        states(i+1,:) = bodyVelsToState(states(i,:),linVel(i),latVel(i),angVel(i),dt(i));
    end
end