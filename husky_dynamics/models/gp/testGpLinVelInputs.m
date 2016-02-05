gpWrapperFname = 'gp_lin_vel_02021657';
gpWrapperName = 'gpLinVel';
datasetFname = '../../data/dataset_gp_lin_vel_1_hold.mat';
fnameOut = [gpWrapperFname '_err'];
vizFlag = 0;

%% input struct
inputStruct.gpWrapperFname = gpWrapperFname;
inputStruct.gpWrapperName = gpWrapperName;
inputStruct.datasetFname = datasetFname;
inputStruct.fnameOut = fnameOut;
inputStruct.vizFlag = vizFlag;
