% inputs
% fnames
gpWrapperFname = 'gp_lat_vel_02021657';
gpWrapperName = 'gpLatVel';
testDatasetFname = '../../data/dataset_gp_lat_vel_1_hold.mat';
testFileType = 'model';
datasetTransformFunc = @dynDatasetToGpLatVelDataset;
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