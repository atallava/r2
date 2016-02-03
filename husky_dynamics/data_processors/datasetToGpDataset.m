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
        [yLinVel,yLatVel,yAngVel] = deal(zeros(nElements,1));
        
        % TODO. any scaling of pose or vels needed?
        for j = 1:nElements
            stateInit = dataset(j).stateInit;
            stateFinal = dataset(j).stateFinal;
            controls = dataset(j).controls;
            dt = dataset(j).dt;
            
            x(j,:) = dynamicsVarsToGpStates(stateInit,controls);
            
            [yLinVel(j),yLatVel(j),yAngVel(j)] = ...
                statesToBodyVels(stateInit,stateFinal,dt);
        end
        
        % store source file name
        sourceFname = fname;
        
        datasetGpFname = strrep(fname,'dataset','dataset_gp_lin_vel');
        y = yLinVel;
        save(datasetGpFname,'x','y','sourceFname');
        fprintf('File saved as %s.\n',datasetGpFname);
        datasetGpFname = strrep(fname,'dataset','dataset_gp_lat_vel');
        y = yLatVel;
        save(datasetGpFname,'x','y','sourceFname');
        fprintf('File saved as %s.\n',datasetGpFname);
        datasetGpFname = strrep(fname,'dataset','dataset_gp_ang_vel');
        y = yAngVel;
        save(datasetGpFname,'x','y','sourceFname');
        fprintf('File saved as %s.\n',datasetGpFname);
    end
end