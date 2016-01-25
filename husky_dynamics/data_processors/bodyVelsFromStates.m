function [linV,angV] = bodyVelsFromStates(stateInit,stateFinal,dt)
    linDiff = norm(stateFinal(1:2)-stateInit(1:2));
    linV = linDiff/dt;
    angDiff = thDiff(stateInit(3),stateFinal(3));
    angV = angDiff/dt;
end