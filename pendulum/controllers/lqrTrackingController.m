function controller = lqrTrackingController(KFinal,stateDesired,controlDesired)
    controller = @(t,state) ...
            controlDesired+KFinal*(state-stateDesired)';
end