function [linV,latV,angV] = statesToBodyVels(stateInit,stateFinal,dt)
    %STATESTOBODYVELS
    %
    % [linV,latV,angV] = STATESTOBODYVELS(stateInit,stateFinal,dt)
    %
    % stateInit  -
    % stateFinal -
    % dt         -
    %
    % linV       - Linear velocity.
    % latV       - Lateral velocity.
    % angV       - Angular velocity.
    
    linDiff = norm(stateFinal(1:2)-stateInit(1:2));
    linV = linDiff/dt;
    angDiff = thDiff(stateInit(3),stateFinal(3));
    angV = angDiff/dt;
end