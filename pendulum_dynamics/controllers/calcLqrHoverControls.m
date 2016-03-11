function controls = calcLqrHoverControls(t,states,K,stateDesired,controlDesired)
    %CALCLQRHOVERCONTROLS
    %
    % controls = CALCLQRHOVERCONTROLS(t,states,K,stateDesired,controlDesired)
    %
    % t              - [nStates,1] array.
    % states         - [nStates,2] array.
    % K              - [2,2] array.
    % stateDesired   - [1,2] array.
    % controlDesired - Scalar.
    %
    % controls       - [nStates,1] array.
    
    states(:,1) = mod(states(:,1),2*pi);
    stateResiduals = bsxfun(@minus,states,stateDesired);
    % K operates on states as columns
    controls = controlDesired+K*stateResiduals';
end