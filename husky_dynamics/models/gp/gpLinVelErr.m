gpLinVelFname = 'gp_lin_vel_01251736';
load(gpLinVelFname,'gpLinVel');
datasetFname = '../../data/dataset_gp_1_hold.mat';
load(datasetFname);
y = y(:,1);

%%
yPred = gpLinVel(x);

%%
errVec = (yPred-y).^2;
fprintf('Mean error: %.3f. std: %.3f.\n',mean(errVec),std(errVec));