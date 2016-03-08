% physical parameters for the pendulum

% mass, kg
physicalParams.m = 1e-1;

% length, m
physicalParams.l = 1;

% gravitational acceleration, m/s
physicalParams.g = 10;

% control noise
physicalParams.controlNoise.variance = 1e4; 

%% save
save('../data/physical_params_noisy','physicalParams');