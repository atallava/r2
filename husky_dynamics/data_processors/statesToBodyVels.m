function [linV,angV] = statesToBodyVels(stateInit,stateFinal,dt)
    %STATESTOBODYVELS
    %
    % [linV,angV] = STATESTOBODYVELS(stateInit,stateFinal,dt)
    %
    % stateInit  -
    % stateFinal -
    % dt         -
    %
    % linV       -
    % angV       -
    
    linDiff = norm(stateFinal(1:2)-stateInit(1:2));
    linV = linDiff/dt;
    angDiff = thDiff(stateInit(3),stateFinal(3));
    angV = angDiff/dt;
end