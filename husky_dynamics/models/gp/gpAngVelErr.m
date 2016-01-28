gpAngVelFname = 'gp_ang_vel_01281605';
load(gpAngVelFname,'gpAngVel');
datasetFname = '../../data/dataset_gp_1_train_subsampled.mat';
load(datasetFname);
y = y(:,2);

%% predict
yPred = gpAngVel(x);

%% errors
errVec = (yPred-y).^2;
fprintf('Mean error: %.3f. std: %.3f.\n',mean(errVec),std(errVec));

%% viz
hf = figure;
plot(y,'.'); hold on;
plot(yPred,'.'); 
legend('meas','pred');
dcm_obj = datacursormode(hf);
set(dcm_obj,'UpdateFcn',@(obj,eventObj) tagPlotPointWithId(obj,eventObj,1:length(yPred),yPred));
