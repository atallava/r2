function varargout = testGp(inputStruct)
    %TESTGP
    %
    % varargout = TESTGP(inputStruct)
    %
    % inputStruct -
    %
    % varargout   - {meanErr,errVec}
    
    %% unpack variables
    % test data
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
    
    % gp wrapper
    if isfield(inputStruct,'gpWrapper')
        gpWrapper = inputStruct.gpWrapper;
    else
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
    end
        
    % save
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
    
    % utils
    if isfield(inputStruct,'vizFlag')
        vizFlag = inputStruct.vizFlag;
    else
        vizFlag = 0;
    end
    if isfield(inputStruct,'dispFlag')
        dispFlag = inputStruct.dispFlag;
    else
        dispFlag = false;
    end
    
    %% load
    if ~exist('gpWrapper','var')
        % load gp wrapper
        tmp = load(gpWrapperFname,gpWrapperName);
        gpWrapper = tmp.(gpWrapperName);
    end
    
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
    
    %% predict
    yPred = gpWrapper(x);
    
    % errors
    errVec = (yPred-y).^2;
    meanErr = mean(errVec);
    stdErr = std(errVec);
    if dispFlag
        fprintf('%s: Mean error: %.3f. std: %.3f.\n',...
            mfilename,meanErr,stdErr);
    end
    
    varargout{1} = meanErr;
    varargout{2} = errVec;
    
    %% viz
    if vizFlag
        hf = figure;
        plot(y,'.'); hold on;
        plot(yPred,'.');
        legend('meas','pred');
        xlabel('data id');
        ylabel('error');
        dcm_obj = datacursormode(hf);
        tagXCell = {1:length(yPred),1:length(yPred)};
        tagYCell = {y,yPred};
        set(dcm_obj,'UpdateFcn',@(obj,eventObj) tagPlotPointWithId(obj,eventObj,tagXCell,tagYCell));
    end
    
    %% save
    if saveRes
        save(testResFname);
    end
end