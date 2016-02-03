function [linVels,latVels,angVels] = statesToBodyVels(statesInit,statesFinal,dts)
    %STATESTOBODYVELS
    %
    % [linV,latV,angV] = STATESTOBODYVELS(statesInit,statesFinal,dts)
    %
    % statesInit  - [nStates,3] array.
    % statesFinal - [nStates,3] array.
    % dts         - [nStates,1] array.
    %
    % linVels       - [nStates,1] array.
    % latVels       - [nStates,1] array.
    % angVels       - [nStates,1] array.
    
    nStates = size(statesInit,1);
    
    % position vectors
    pi = statesInit(:,1:2); 
    pf = statesFinal(:,1:2);
    dp = pf-pi; % [nStates,2]
    
    % orientation vectors
    thi = statesInit(:,3);
    thf = statesFinal(:,3);
    qi = [cos(thi) sin(thi)]; % [nStates,2]
    qf = [cos(thf) sin(thf)];  % [nStates,2]
    
    angVels = thDiff(thi,thf)/dts;
    
    qfRowNorms = sqrt(sum(qf.^2,2));
    r = bsxfun(@rdivide,qf,qfRowNorms);  % simply qf translated to pi
        
    k = repmat([0 0 1],nStates,1); % z-vector. [nStates,3]
    r3 = [r zeros(nStates,1)]; % [nStates,3]
    % lateral velocity support vector
    s = cross(k,r3); s = s(:,1:2); % [nStates,2]
    
    dpPll = bsxfun(@times,r,dot(dp,r,2));
    dpPerp = dp-dpPll;
    linVels = dot(dp,r,2)/dts;
    latVels = dot(dpPerp,s,2)/dts;        
end