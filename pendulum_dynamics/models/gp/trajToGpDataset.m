function [x,y] = trajToGpDataset(fnameTrajectory)
    %TRAJTOGPDATASET
    %
    % [x,y] = TRAJTOGPDATASET(fnameTrajectory)
    %
    % fnameTrajectory -
    %
    % x               - [nElements,1] array. Theta.
    % y               - [nElements,1] array. Theta ddot.
    
    load(fnameTrajectory,'physicalParams','controller','dt','state0','nSteps',...
        't','states','statesDot','controls');
    
    % x is pendulum theta
    x = [state0(1); states(1:end-1,1)];
    % y is acceleration
    y = statesDot(:,2);
    
end