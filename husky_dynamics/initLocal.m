someUsefulPaths
addpath(genpath([pathToM '/at_utils_m']));

% for gp 
huskyDynamicsPath = pwd;
cd([pathToM '/gpml-matlab-v3.6-2015-07-07']);
run('startup');
cd(huskyDynamicsPath);

% for various helpers
addpath(genpath([pathToM '/neato_utils']));

addpath([huskyDynamicsPath '/data_processors']);




