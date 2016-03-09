% physical parameters
load('../data/physical_params','physicalParams');

% linearization point
stateDesired = [pi 0];
controlDesired = 0;

% sim time step
dt = 1e-3;

% dynamics terms
[A,B,ABResidual] = linearizeAboutDesired(stateDesired,controlDesired,dt,physicalParams);

% cost terms
Q = [1 0; ...
    0 1];
R = 0.01;

%% save
fnameLinearization = '../data/linearization_terms';
save(fnameLinearization,'stateDesired','controlDesired',....
    'dt','A','B','ABResidual',...
    'Q','R');

