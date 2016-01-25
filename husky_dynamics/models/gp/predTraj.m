function [states,linVel,angVel] = predTraj(gpLinVel,gpAngVel,stateInit,controls,dt)
    %GENTRAJECTORY
    %
    % [states,linVel,angVel] = GENTRAJECTORY(gpVelParams,gpAngParams,stateInit,controls,dt)
    %
    % gpLinVel - Function handle.
    % gpAngVel - Function handle.
    % stateInit   - Length 3 vector.
    % controls    - [nMeasurements,2] array.
    % dt          - nMeasurements length vector.
    %
    % states      - [nMeasurements+1,3] array.
    % linVel      - nMeasurements length vector.
    % angVel      - nMeasurements length vector.
    
    nTimesteps = length(dt);
    dimState = length(stateInit);
    dimControls = size(controls,2);
    states = zeros(nTimesteps+1,dimState);
    states(1,:) = stateInit;
    linVel = zeros(1,nTimesteps);
    angVel = zeros(1,nTimesteps);
    xQuery = zeros(nTimesteps,dimState+dimControls);
        
    for i = 1:nTimesteps
        xQuery(i,:) = [states(i,:) controls(i,:)];
        linVel(i) = gpLinVel(xQuery(i,:));
        angVel(i) = gpAngVel(xQuery(i,:));
        % euler integration
        states(i+1,3) = states(i,3)+angVel(i)*dt(i);
        states(i+1,1) = states(i,1)+linVel(i)*cos(states(i+1,3))*dt(i);
        states(i+1,2) = states(i,2)+linVel(i)*sin(states(i+1,3))*dt(i);
    end
end