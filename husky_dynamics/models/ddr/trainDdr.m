function WOptim = trainDdr(inputStruct)
    
    if isfield(inputStruct,'trainDatasetFname')
        trainDatasetFname = inputStruct.trainDatasetFname;
    else
        error('trainDatasetFname not a field in inputStruct.');
    end

    load(fnameTrain,'dataset');
    
    
    fun = @(W) predictionRisk(dataset.statesFinal,predict(W,dataset));
    lb = 1e-6;
    ub = 5;
    W0 = 1;
    [WOptim,objOptim,optExitflag,optOutput] = fmincon(fun,W0,[],[],[],[],lb,ub);
end
