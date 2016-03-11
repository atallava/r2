% physical params
fnamePhysicalParams = '../../data/traj/physical_params';
load(fnamePhysicalParams,'physicalParams');

% zero control
controller = @(t,states) calcConstantControls(t,states,0);

%% forward simulate
dt = 0.01;
stateInit = [deg2rad(90) 0.0];
nSteps = 2000;

[t,states,controls,statesDot] = forwardSimulateODEWithController(stateInit,controller,dt,nSteps,physicalParams);

%% viz system
hfig1 = vizStates(states,physicalParams);

%% plot state
hfig2 = figure;
plot(t,states(:,1));
title('theta');

hfig3 = figure;
plot(t,states(:,2));
title('theta_dot');

%% save trajectory
fnameTraj = '../../data/traj/uncontrolled_pendulum_traj';
save(fnameTraj,'physicalParams','controller',...
    'dt','stateInit','nSteps',...
    't','states','controls','statesDot');