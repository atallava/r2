% physical parameters for the pendulum
% mass, kg
physicalParams.m = 1e-1;
% length, m
physicalParams.l = 1;
% gravitational acceleration, m/s
physicalParams.g = 10;

%% save
save('../data/physical_params','physicalParams');