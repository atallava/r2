function controller = lqrTrackingController(KFinal,stateDesired,controlDesired)
    %LQRTRACKINGCONTROLLER
    %
    % controller = LQRTRACKINGCONTROLLER(KFinal,stateDesired,controlDesired)
    %
    % KFinal         -
    % stateDesired   -
    % controlDesired -
    %
    % controller     -

    controller = @(t,state) ...
            controlDesired+KFinal*(state-stateDesired)';
end