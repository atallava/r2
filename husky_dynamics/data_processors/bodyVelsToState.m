function [statesFinal] = bodyVelsToState(statesInit,linVels,latVels,angVels,dts)
    %BODYVELSTOSTATE
    %
    % [statesFinal] = BODYVELSTOSTATE(statesInit,linVels,latVels,angVels,dts)
    %
    % statesInit - [nStates,3] array.
    % linVels    - [nStates,1] array.
    % latVels    - [nStates,1] array.
    % angVels    - [nStates,1] array.
    % dts        - [nStates,1] array.
    %
    % statesFinal - [nStates,3] array.
    
    nStates = size(statesInit,1);
    
    pi = statesInit(:,1:2); % [nStates,2]
    thi = statesInit(:,3); 
    thf = thi+angVels.*dts; 
    r = [cos(thf) sin(thf)]; % [nStates,2]
    r3 = [r zeros(nStates,1)]; % [nStates,3]
    k = repmat([0 0 1],nStates,1); % [nStates,3]
    s = cross(k,r3); s = s(:,1:2); % [nStates,2]
    dpPll = bsxfun(@times,r,dts.*linVels); % [nStates,2]
    dpPerp = bsxfun(@times,s,dts.*latVels); % [nStates,2]
    dp = dpPll+dpPerp;
    pf = pi+dp;
    statesFinal = [pf thi];
    
%     pi = stateInit(1:2); pi = flipVecToColumn(pi);
%     thi = stateInit(3);
%     thf = thi+angVel*dt;
%     r = [cos(thf); sin(thf)];
%     k = [0; 0; 1];
%     s = cross(k,[r; 0]); s = s(1:2);
%     dpPll = dt*linVel.*r;
%     dpPerp = dt*latVel.*s;
%     dp = dpPll+dpPerp;
%     pf = pi+dp;
%     stateFinal = [pf' thf];    
end