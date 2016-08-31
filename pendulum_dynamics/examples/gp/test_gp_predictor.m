% inputs
% fnames
gpWrapperFname = '../../data/gp/gp_predictor_04212146';
gpWrapperName = 'gpPredictor';
testDatasetFname = '../../data/gp/dataset_gp_1_train.mat';
testFileType = 'model';
datasetTransformFunc = @dynDatasetToGpDataset;
saveRes = 0;
testResFname = [gpWrapperFname '_test'];
vizFlag = 1;

%% pack to struct
inputStruct.gpWrapperFname = gpWrapperFname;
inputStruct.gpWrapperName = gpWrapperName;
inputStruct.testDatasetFname = testDatasetFname;
inputStruct.testFileType = testFileType;
inputStruct.datasetTransformFunc = datasetTransformFunc;
inputStruct.saveRes = saveRes;
inputStruct.testResFname = testResFname;
inputStruct.vizFlag = vizFlag;

%% test
testGp(inputStruct);