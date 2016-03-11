% physical params
fnamePhysicalParams = '../../data/traj/physical_params';
load(fnamePhysicalParams,'physicalParams');

% controller
fnameController = '../../data/traj/lqr_hover_controller';
load(fnameController,'controller');

%% forward simulate for some time
dt = 0.01;
nSteps = 1000;
stateInit = [pi+deg2rad(-70) 0.0];

[t,states,controls,stateDot] = forwardSimulateODEWithController(stateInit,controller,dt,nSteps,physicalParams);

%% viz system
hfig1 = vizStates(states,physicalParams);

%% plot state
hfig2 = figure;
plot(t,states(:,1));
title('theta');

hfig3 = figure;
plot(t,states(:,2));
title('theta_dot');

%% save
fnameTraj = '../../data/traj/lqr_hover_traj';
save(fnameTraj,'physicalParams','controller',...
    'dt','stateInit','nSteps',...
    't','states','controls','statesDot');