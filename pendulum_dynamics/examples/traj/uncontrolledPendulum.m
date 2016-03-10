% physical params
fnamePhysicalParams = '../data/physical_params';
load(fnamePhysicalParams,'physicalParams');

% controller
controller = constantController(0);

%% forward simulate for some time
dt = 0.01;

state0 = [deg2rad(90) 0.0];

nSteps = 2000;
[t,states,statesDot,controls] = forwardSimulateODE(state0,controller,dt,nSteps,physicalParams);

%% viz system
hfig1 = vizStates([state0; states],physicalParams);

%% plot state
hfig2 = figure;
plot(t,states(:,1));
title('theta');

hfig3 = figure;
plot(t,states(:,2));
title('theta_dot');

%% save trajectory
fnameTrajectory = '../data/uncontrolled_pendulum_trajectory';
save(fnameTrajectory,'physicalParams','controller','dt','state0','nSteps',...
    't','states','statesDot','controls');