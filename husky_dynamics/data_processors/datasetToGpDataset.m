function datasetToGpDataset(fnames)
    %DATASETTOGPDATASET
    %
    % DATASETTOGPDATASET(fnames)
    %
    % fnames - String or cell of strings.
    
    if ~iscell(fnames)
        fnames = {fnames};
    end
    
    for i = 1:length(fnames)
        fname = fnames{i};
        load(fname,'dataset');
        
        nElements = length(dataset);
        x = zeros(nElements,5);
        y = zeros(nElements,2);
        
        % any scaling of pose or vels needed?
        for j = 1:nElements
            stateInit = dataset(j).stateInit;
            stateFinal = dataset(j).stateFinal;
            controls = dataset(j).controls;
            dt = dataset(j).dt;
            
            x(j,1:3) = stateInit;
            x(j,4:5) = controls;
            [y(j,1),y(j,2)] = bodyVelsFromStates(stateInit,stateFinal,dt);
        end
        % project angles to [0,2*pi]
        x(:,3) = mod(x(:,3),2*pi);
        
        datasetGpFname = strrep(fname,'dataset','dataset_gp');
        save(datasetGpFname,'x','y');
    end
end