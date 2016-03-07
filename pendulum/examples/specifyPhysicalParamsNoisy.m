% physical parameters for the pendulum

% mass, kg
physicalParams.m = 1e-1;

% length, m
physicalParams.l = 1;

% gravitational acceleration, m/s
physicalParams.g = 10;

% motion noise
physicalParams.motionNoise = [deg2rad(1e4)^2 0; ...
    0 deg2rad(1e4)^2];

%% save
save('../data/physical_params_noisy','physicalParams');