% load
% physical params
fnamePhysicalParams = '../../data/traj/physical_params';
load(fnamePhysicalParams,'physicalParams');

% predictor 
predictorFname = '../../data/gp/gp_predictor_03101431';
load(predictorFname,'gpPredictor');

%% forward simulate
% predictor struct
predictorStruct.predictor = gpPredictor;
predictorStruct.dynStateToPredictorState = @transformDynStateToGpState;

stateInit = [deg2rad(10) 0];
dt = 0.01;
nSteps = 1000;
[t,states,statesDot] = forwardSimulateODEWithPredictor(stateInit,dt,nSteps,predictorStruct);

%% viz system
hfig1 = vizStates(states,physicalParams);

%% plot states
hfig2 = figure;
plot(t,states(:,1));
title('theta');

hfig3 = figure;
plot(t,states(:,2));
title('theta_dot');
