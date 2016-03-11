function testGp(inputStruct)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % unpack variables
    if isfield(inputStruct,'gpWrapperFname')
        gpWrapperFname = inputStruct.gpWrapperFname;
    else
        error('gpWrapperFname not a field in inputStruct.');
    end
    if isfield(inputStruct,'gpWrapperName')
        gpWrapperName = inputStruct.gpWrapperName;
    else
        error('gpWrapperName not a field in inputStruct.');
    end
    if isfield(inputStruct,'testDatasetFname')
        testDatasetFname = inputStruct.testDatasetFname;
    else
        error('testDatasetFname not a field in inputStruct.');
    end
    if isfield(inputStruct,'testFileType')
        testFileType = inputStruct.testFileType;
    else
        error('testFileType not input.');
    end
    if strcmp(testFileType,'dyn')
        if isfield(inputStruct,'datasetTransformFunc')
            datasetTransformFunc = inputStruct.datasetTransformFunc;
        else
            error('datasetTransformFunc not input.');
        end
    end
    if isfield(inputStruct,'saveRes')
        saveRes = inputStruct.saveRes;
    else
        saveRes = 0;
    end
    if saveRes
        if isfield(inputStruct,'testResFname')
            testResFname = inputStruct.testResFname;
        else
            error('testResFname not input.');
        end
    end
    if isfield(inputStruct,'vizFlag')
        vizFlag = inputStruct.vizFlag;
    else
        vizFlag = 0;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % load gp wrapper
    tmp = load(gpWrapperFname,gpWrapperName);
    gpWrapper = tmp.(gpWrapperName);
    
    % load test dataset
    switch testFileType
        case 'model'
            load(testDatasetFname,'x','y');
        case 'dyn'
            load(testDatasetFname,'dataset');
            [x,y] = datasetTransformFunc(dataset);
        otherwise
            error('testGp:invalidInput','testFileType must be dyn or model');
    end
    
    % predict
    yPred = gpWrapper(x);
    
    % errors
    errVec = (yPred-y).^2;
    meanErr = mean(errVec);
    stdErr = std(errVec);
    fprintf('Mean error: %.3f. std: %.3f.\n',meanErr,stdErr);
    
    % viz
    if vizFlag
        hf = figure;
        plot(y,'.'); hold on;
        plot(yPred,'.');
        legend('meas','pred');
        dcm_obj = datacursormode(hf);
        set(dcm_obj,'UpdateFcn',@(obj,eventObj) tagPlotPointWithId(obj,eventObj,1:length(yPred),yPred));
    end
    
    % save
    if saveRes
        save(testResFname);
    end
end