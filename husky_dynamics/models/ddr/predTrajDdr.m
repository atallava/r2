function states = predTrajDdr(stateInit,controls,dt,wheelBase)
    %PREDTRAJDDR
    %
    % [states,linVel,latVel,angVel] = PREDTRAJDDR(stateInit,controls,dt,gpLinVel,gpLatVel,gpAngVel)
    %
    % stateInit   - Length 3 vector.
    % controls    - [nMeasurements,2] array.
    % dt          - nMeasurements length vector.
    % wheelBase   - Scalar.
    %
    % states      - [nMeasurements+1,3] array.
    
    [~,states] = fwdSim(stateInit,controls,dt,wheelBase);
end