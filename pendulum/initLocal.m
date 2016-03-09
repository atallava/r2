someUsefulPaths
addpath(genpath([pathToM '/at_utils_m']));

pendulumPath = pwd;
addpath([pendulumPath '/controllers']);
addpath([pendulumPath '/misc']);
addpath(genpath([pendulumPath '/models']));
addpath([pendulumPath '/viz']);

% for gp models
utilsPath = [pwd '/../utils'];
addpath([utilsPath '/models/gp']);
