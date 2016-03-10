function [x,y] = dynDatasetToGpDataset(dataset)
    %DYNDATASETTOGPDATASET
    %
    % [x,y] = DYNDATASETTOGPDATASET(dataset)
    %
    % dataset - Dynamics dataset struct.
    %
    % x       - [nElements,1] array. Theta.
    % y       - [nElements,1] array. Theta ddot.
    
    nElements = length(dataset);
    
    % extract statesInit
    statesInit = [dataset.stateInit];
    statesInit = reshape(statesInit,2,numel(statesInit)/2); % [2,nElements]
    statesInit = statesInit'; % [nElements,2];
    
    % extract statesDot
    % ASSUMING: 1-step transitions
    statesDot = [dataset.stateInit];
    statesDot = reshape(statesDot,2,numel(statesDot)/2); % [2,nElements]
    statesDot = statesDot'; % [nElements,2];
    
    x = statesInit(:,1);
    y = statesDot(:,2);
end