% inputs
% fnames
trainDatasetFname = '../../data/gp/dataset_gp_1_train';
trainFileType = 'model';
datasetTransformFunc = @dynDatasetToGpDataset;
gpWrapperFname = ['../../data/gp/gp_predictor' '_' myDateStamp(2:5)] ;

%% gp params
dimState = 1;

infMethod = @infExact;

meanFunc = {@meanConst};
hyp.mean = 0;

covFunc = {@covSEard}; 
sf = 2;
vecArd = rand(dimState,1);
hyp.cov = log([vecArd; sf]);

likFunc = @likGauss;
hyp.lik = log(0.1);

% for hyp optimization
maxIterations = 50;

%% gp wrapper name
gpWrapperName = 'gpPredictor';

%% pack into struct
inputStruct.trainDatasetFname = trainDatasetFname;
inputStruct.trainFileType = trainFileType;
inputStruct.gpWrapperFname = gpWrapperFname;
inputStruct.infMethod = infMethod;
inputStruct.meanFunc = meanFunc;
inputStruct.covFunc = covFunc;
inputStruct.hyp = hyp;
inputStruct.likFunc = likFunc;
inputStruct.maxIterations = maxIterations;
inputStruct.gpWrapperName = gpWrapperName;

%% train
trainGp(inputStruct);