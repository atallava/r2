% load physical params
load('physical_params','physicalParams');

% constant torque forward sim
tq = 0;

% start state
stateInit = deg2rad([0 1]);

% simulation time step
dt = 1e-2;

% steps
nSteps = 1e3;

%% forward simulate
controls = repmat(tq,nSteps,1);
[t,states] = forwardSimulate(stateInit,controls,dt,physicalParams);

% sanity check, energy should be constant
[te,ke,pe] = calcEnergy(states,physicalParams);