function x = dynStatesToGpStates(states,controls)
    %DYNSTATESTOGPSTATES
    %
    % x = DYNSTATESTOGPSTATES(states,controls)
    %
    % states   - [nStates,3] array.
    % controls - [nStates,2] array.
    %
    % x        - [nStates,5] array.

    nStates = size(states,1);
    condn = nStates == size(controls,1);
    assert(condn,'dynamicsVarsToGpState:invalidInput',...
        'Number of states must equal number of controls.');
    x = zeros(nStates,5);
    % snap orientations to [0,2pi]
    states(:,3) = mod(states(:,3),2*pi);
    x = [states controls];
end