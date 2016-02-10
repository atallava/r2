function [x,y] = dynDatasetToGpLinVelDataset(dataset)
    %DYNDATASETTOGPLINVELDATASET
    %
    % [x,y] = DYNDATASETTOGPLINVELDATASET(dataset)
    %
    % dataset - Struct array with fields
    % ('stateInit','stateFinal','controls','dt')
    %
    % x       - [nElements,5] array.
    % y       - [nElements,1] array.

    nElements = length(dataset);
    x = zeros(nElements,5);
    y = zeros(nElements,1);
    
    for j = 1:nElements
        stateInit = dataset(j).stateInit;
        stateFinal = dataset(j).stateFinal;
        controls = dataset(j).controls;
        dt = dataset(j).dt;
        
        x(j,:) = dynStatesToGpStates(stateInit,controls);
        
        [y(j),~,~] = statesToBodyVels(stateInit,stateFinal,dt);
    end
end