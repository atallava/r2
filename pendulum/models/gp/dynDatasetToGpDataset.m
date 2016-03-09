function [x,y] = trajDatasetToGpDataset(fnameTrajectory)
    load(fnameTrajectory,'physicalParams','controller','dt','state0','nSteps',...
        't','states','statesDot','controls');
    
    % x is pendulum theta
    x = [state0(1); states(1:end-1,1)];
    % y is acceleration
    y = statesDot(:,2);
    
end