someUsefulPaths
addpath(genpath([pathToM '/at_utils_m']));

pendulumPath = pwd;
addpath([pendulumPath '/controllers']);
addpath([pendulumPath '/misc']);
addpath([pendulumPath '/models']);
addpath([pendulumPath '/viz']);

% for gp models
huskyDynamicsPath = [pwd '/../husky_dynamics'];
addpath([huskyDynamicsPath '/models/gp']);
