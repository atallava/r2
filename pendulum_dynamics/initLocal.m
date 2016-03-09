someUsefulPaths
addpath(genpath([pathToM '/at_utils_m']));

pendulumDynamicsPath = pwd;
addpath([pendulumDynamicsPath '/controllers']);
addpath([pendulumDynamicsPath '/data_processors']);
addpath([pendulumDynamicsPath '/misc']);
addpath(genpath([pendulumDynamicsPath '/models']));
addpath([pendulumDynamicsPath '/viz']);

% for gp 
cd([pathToM '/gpml-matlab-v3.6-2015-07-07']);
run('startup');
cd(pendulumDynamicsPath);

% for gp models
utilsPath = [pwd '/../utils'];
addpath([utilsPath '/models/gp']);
