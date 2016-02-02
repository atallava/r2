gpLinVelFname = 'gp_lin_vel_02021657';
load(gpLinVelFname,'gpLinVel');
datasetFname = '../../data/dataset_gp_lin_vel_1_hold.mat';
load(datasetFname);

%% predict
yPred = gpLinVel(x);

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
