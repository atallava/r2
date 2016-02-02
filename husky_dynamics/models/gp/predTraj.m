function [states,linVel,latVel,angVel] = predTraj(gpLinVel,gpLatVel,gpAngVel,stateInit,controls,dt)
    %GENTRAJECTORY
    %
    % [states,linVel,angVel] = GENTRAJECTORY(gpVelParams,gpAngParams,stateInit,controls,dt)
    %
    % gpLinVel - Function handle.
    % gpAngVel - Function handle.
    % gpLatVel - Function handle.
    % stateInit   - Length 3 vector.
    % controls    - [nMeasurements,2] array.
    % dt          - nMeasurements length vector.
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
        xQuery(i,:) = [states(i,:) controls(i,:)];
        linVel(i) = gpLinVel(xQuery(i,:));
        latVel(i) = gpLatVel(xQuery(i,:));
        angVel(i) = gpAngVel(xQuery(i,:));
        states(i+1,:) = bodyVelsToState(states(i,:),linVel(i),latVel(i),angVel(i),dt(i));
    end
end