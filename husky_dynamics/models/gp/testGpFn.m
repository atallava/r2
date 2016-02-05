function testGpFn(inputStruct)
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
    if isfield(inputStruct,'datasetFname')
        datasetFname = inputStruct.datasetFname;
    else
        error('datasetFname not a field in inputStruct.');
    end
    if isfield(inputStruct,'fnameOut')
        fnameOut = inputStruct.fnameOut;
    else
        fnameOut = [];
    end
    if isfield(inputStruct,'vizFlag')
        vizFlag = inputStruct.vizFlag;
    else
        vizFlag = 0;
    end
    
    % run script
    testGpScript;
end