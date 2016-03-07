% physical params
fnamePhysicalParams = '../data/physical_params_noisy';
load(fnamePhysicalParams,'physicalParams');

% controller
fnameController = '../data/lqr_tracking_controller';
load(fnameController,'controller');

%% forward simulate for some time
dt = 0.01;

state0 = [pi+deg2rad(0) 0];

nSteps = 2000;
[t,states,controls] = forwardSimulateODE(state0,controller,dt,nSteps,physicalParams);

%% viz system
hfig1 = vizStates([state0; states],physicalParams);

%% plot state
hfig2 = figure;
plot(t,states(:,1));
title('theta');

hfig3 = figure;
plot(t,states(:,2));
title('theta_dot');