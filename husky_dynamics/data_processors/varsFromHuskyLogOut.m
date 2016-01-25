function [state,controls,t] = varsFromHuskyLogOut(huskyLogStruct)
    %VARSFROMHUSKYLOGOUT One more processing layer.
    %
    % [state,controls,t] = VARSFROMHUSKYLOGOUT(huskyLogStruct)
    %
    % huskyLogStruct - Output of parseHuskyLog.m
    %
    % state          - [nMeasurements,3] array. [x,y,theta].
    % controls       - [nMeasurements,2] array. [velLeft,velRight].
    % t              - nMeasurements length vector. Timestamps.

    nMeasurements = length(huskyLogStruct.t);
    state = zeros(nMeasurements,3);
    state(:,1:2) = huskyLogStruct.xyz(:,1:2);
    state(:,3) = huskyLogStruct.rpy(:,end);
    controls = zeros(nMeasurements,2);
    controls(:,1) = huskyLogStruct.velLeft;
    controls(:,2) = huskyLogStruct.velRight;
    t = huskyLogStruct.t;
end