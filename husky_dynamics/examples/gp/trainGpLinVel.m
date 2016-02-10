% inputs
% fnames
trainDatasetFname = '../../data/dataset_gp_lin_vel_1_train_subsampled';
trainFileType = 'model';
datasetTransformFunc = @dynDatasetToGpLinVelDataset;
gpWrapperFname = ['data/gp_lin_vel' '_' myDateStamp(2:5)] ;

%% gp params
infMethod = @infExact;

meanFunc = {@meanConst};
hyp.mean = 0;

covFunc = {@covMaternard,5}; 
sf = 2;
vecArd = rand(dimState,1);
hyp.cov = log([vecArd; sf]);

likFunc = @likGauss;
hyp.lik = log(0.1);

% for hyp optimization
maxIterations = 10;

%% gp wrapper name
gpWrapperName = 'gpLinVel';

%% pack into struct
inputStruct.trainDatasetFname = trainDatasetFname;
inputStruct.trainFileType = trainFileType;
inputStruct.datasetTransformFunc = datasetTransformFunc;
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
