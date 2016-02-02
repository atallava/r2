function [linVel,latVel,angVel] = statesToBodyVels(stateInit,stateFinal,dt)
    %STATESTOBODYVELS
    %
    % [linV,latV,angV] = STATESTOBODYVELS(stateInit,stateFinal,dt)
    %
    % stateInit  - Length 3 vector.
    % stateFinal - Length 3 vector.
    % dt         - Scalar.
    %
    % linV       - Linear velocity.
    % latV       - Lateral velocity.
    % angV       - Angular velocity.
    
    % position vectors
    pi = stateInit(1:2); pi = flipVecToColumn(pi);
    pf = stateFinal(1:2); pf = flipVecToColumn(pf);
    dp = pf-pi;
    
    % orientation vectors
    thi = stateInit(3);
    thf = stateFinal(3);
    qi = [cos(thi); sin(thi)];
    qf = [cos(thf); sin(thf)];
    
    angVel = thDiff(thi,thf)/dt;
    
    r = qf./norm(qf); % simply qf translated to pi
    k = [0; 0; 1]; % z-vector
    s = cross(k,[r; 0]); s = s(1:2); 
    dpPll = dot(dp,r).*r;
    dpPerp = dp-dpPll;
    linVel = dot(dp,r)/dt;
    latVel = dot(dpPerp,s)/dt;
end