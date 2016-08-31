function varargout = trainGp(inputStruct)
    %TRAINGP
    %
    % varargout = TRAINGP(inputStruct)
    %
    % inputStruct -
    %
    % varargout   - {gpWrapper}
    
    %% unpack variables
    
    % training data
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
    
    % gp options
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

    % wrapper options
    if isfield(inputStruct,'gpWrapperName')
        gpWrapperName = inputStruct.gpWrapperName;
    else
        error('gpWrapperName not input.');
    end
    if isfield(inputStruct,'saveRes')
        saveRes = inputStruct.saveRes;
    else
        saveRes = 0;
    end
    if saveRes
        if isfield(inputStruct,'gpWrapperFname')
            gpWrapperFname = inputStruct.gpWrapperFname;
        else
            error('gpWrapperFname not input.');
        end
    end    
    if isfield(inputStruct,'dispFlag')
        dispFlag = inputStruct.dispFlag;
    else
        dispFlag = false;
    end
    
    %% load training dataset
    switch trainFileType
        case 'model'
            load(trainDatasetFname,'x','y');
        case 'dyn'
            load(trainDatasetFname,'dataset');
            [x,y] = datasetTransformFunc(dataset);
        otherwise
            error('trainGp:invalidInput','trainFileType must be dyn or model');
    end
    
    %% optimize hyperparams
    if dispFlag
        fprintf('%s: training %s.\n',mfilename,gpWrapperName);
    end
    clockLocal = tic();
    
    % optimize on training nll
    fun = @(hyp) gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y);
    
    hyp = minimize(hyp,fun,-maxIterations);
    tComp = toc(clockLocal);
    
    if dispFlag
        fprintf('%s: computation time: %.3f.\n',mfilename,tComp);
    end
    nll = gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y);
    
    if dispFlag
        fprintf('%s: training data nll: %.3f.\n',mfilename,nll);
    end
    
    %% create gp wrapper function
    cmd = sprintf('%s = @(xQuery) gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y,xQuery);',gpWrapperName);
    eval(cmd);
    
    %% save
    if saveRes
        save(gpWrapperFname);
        if dispFlag
            fprintf('%s: results saved in %s.\n',mfilename,gpWrapperFname);
        end
    end    
    
    %% output gpWrapper
    varargout{1} = eval(gpWrapperName);
end