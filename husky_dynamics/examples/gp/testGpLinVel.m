% inputs
% fnames
gpWrapperFname = 'data/gp_lin_vel_02021657';
gpWrapperName = 'gpLinVel';
testDatasetFname = '../../data/dataset_gp_lin_vel_1_hold.mat';
testFileType = 'model';
datasetTransformFunc = @dynDatasetToGpLinVelDataset;
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