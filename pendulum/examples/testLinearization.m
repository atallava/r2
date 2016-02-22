% physical parameters
load('../data/physical_params','physicalParams');

% linearization terms
load('../data/linearization_terms','stateDesired','dt','A','B','ABResidual');

%% compare sims
% slightly perturbed state
stateInit = stateDesired+0.1*deg2rad(rand(1,2));

% random control
controls = 0.1*rand();

% fwd simulate
[~,stateOde] = forwardSimulateODE(stateInit,controls,dt,physicalParams);
[~,stateAB] = forwardSimulateAB(stateInit,controls,dt,A,B,ABResidual);

% print
fprintf('fwd sim ode:\n');
disp(stateOde);
fprintf('fwd sim ab:\n');
disp(stateAB);