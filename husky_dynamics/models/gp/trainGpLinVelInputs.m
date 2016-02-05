% filenames
fnameTrain = '../../data/dataset_gp_lin_vel_1_train_subsampled';
gpWrapperFname = ['gp_lin_vel' '_' myDateStamp(2:5)] ;

%% gp params
infMethod = @infExact;
meanFunc = [];
covFunc = @covSEiso; 
hyp.cov = [0; 0];
likFunc = @likGauss;
hyp.lik = log(0.1);

maxIterations = 50;

%% gp fn name
gpWrapperName = 'gpLinVel';

%% input struct
inputStruct.fnameTrain = fnameTrain;
inputStruct.gpWrapperFname = gpWrapperFname;
inputStruct.infMethod = infMethod;
inputStruct.meanFunc = meanFunc;
inputStruct.covFunc = covFunc;
inputStruct.hyp = hyp;
inputStruct.likFunc = likFunc;
inputStruct.maxIterations = maxIterations;
inputStruct.gpWrapperName = gpWrapperName;
