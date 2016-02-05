function trainGpFn(inputStruct)
    % extract variables
    if isfield(inputStruct,'fnameTrain')
        fnameTrain = inputStruct.fnameTrain;
    else
        error('fnameTrain not input.');
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
    
    % run script
    trainGpScript;
end