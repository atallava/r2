% inputs
% fnames
gpWrapperFname = '../../data/gp/gp_predictor_03091714';
gpWrapperName = 'gpPredictor';
testDatasetFname = '../../data/gp/dataset_gp_1_hold.mat';
testFileType = 'model';
datasetTransformFunc = @dynDatasetToGpDataset;
testResFname = [gpWrapperFname '_test'];
vizFlag = 0;

%% pack to struct
inputStruct.gpWrapperFname = gpWrapperFname;
inputStruct.gpWrapperName = gpWrapperName;
inputStruct.testDatasetFname = testDatasetFname;
inputStruct.testFileType = testFileType;
inputStruct.datasetTransformFunc = datasetTransformFunc;
inputStruct.testResFname = testResFname;
inputStruct.vizFlag = vizFlag;

%% test
testGp(inputStruct);