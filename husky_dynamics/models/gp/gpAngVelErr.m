gpAngVelFname = 'gp_ang_vel_01251740';
load(gpAngVelFname,'gpAngVel');
datasetFname = '../../data/dataset_gp_1_hold.mat';
load(datasetFname);
y = y(:,2);

%%
yPred = gpAngVel(x);

%%
errVec = (yPred-y).^2;
fprintf('Mean error: %.3f. std: %.3f.\n',mean(errVec),std(errVec));