function trainGp(inputStruct)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % unpack variables
    if isfield(inputStruct,'trainDatasetFname')
        trainDatasetFname = inputStruct.trainDatasetFname;
    else
        error('trainDatasetFname not input.');
    end
    if isfield(inputStruct,'trainFileType')
        trainFileType = inputStruct.trainFileType;
    else
        error('trainFileType not input.');
    end
    if strcmp(trainFileType,'dyn')
        if isfield(inputStruct,'datasetTransformFunc')
            datasetTransformFunc = inputStruct.datasetTransformFunc;
        else
            error('datasetTransformFunc not input.');
        end
    end
    if isfield(inputStruct,'gpWrapperFname')
        gpWrapperFname = inputStruct.gpWrapperFname;
    else
        error('gpWrapperFname not input.');
    end
    if isfield(inputStruct,'infMethod')
        infMethod = inputStruct.infMethod;
    else
        error('infMethod not input.');
    end
    if isfield(inputStruct,'meanFunc')
        meanFunc = inputStruct.meanFunc;
    else
        error('meanFunc not input.');
    end
    if isfield(inputStruct,'covFunc')
        covFunc = inputStruct.covFunc;
    else
        error('covFunc not input.');
    end
    if isfield(inputStruct,'hyp')
        hyp = inputStruct.hyp;
    else
        error('hyp not input.');
    end
    if isfield(inputStruct,'likFunc')
        likFunc = inputStruct.likFunc;
    else
        error('likFunc not input.');
    end
    if isfield(inputStruct,'maxIterations')
        maxIterations = inputStruct.maxIterations;
    else
        error('maxIterations not input.');
    end
    if isfield(inputStruct,'gpWrapperName')
        gpWrapperName = inputStruct.gpWrapperName;
    else
        error('gpWrapperName not input.');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % load training dataset
    switch trainFileType
        case 'model'
            load(trainDatasetFname,'x','y');
        case 'dyn'
            load(trainDatasetFname,'dataset');
            [x,y] = datasetTransformFunc(dataset);
        otherwise
            error('trainGp:invalidInput','trainFileType must be dyn or model');
    end
    
    % optimize hyperparams
    fprintf('Training %s.\n',gpWrapperName);
    clockLocal = tic();
    
    % optimize on training nll
    fun = @(hyp) gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y);
    
    hyp = minimize(hyp,fun,-maxIterations);
    tComp = toc(clockLocal);
    fprintf('Computation time: %.3f.\n',tComp);
    nll = gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y);
    fprintf('Training data nll: %.3f.\n',nll);
    
    % create gp wrapper function
    line = sprintf('%s = @(xQuery) gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y,xQuery);',gpWrapperName);
    eval(line);
    
    % save
    save(gpWrapperFname);
end