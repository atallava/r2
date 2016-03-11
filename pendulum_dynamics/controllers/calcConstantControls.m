function controls = calcConstantControls(t,states,c)
    %CALCCONSTANTCONTROLS
    %
    % controls = CALCCONSTANTCONTROLS(t,states,c)
    %
    % t        - [nStates,1] array.
    % states   - [nStates,dimState] array.
    % c        - Scalar.
    %
    % controls - [nStates,1] array.
    
    nStates = size(states,1);
    condn = length(t) == nStates;
    assert(condn,'Number of timesteps should equal number of states. Failed check.');
    
    controls = ones(nStates,1)*c;
end