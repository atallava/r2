someUsefulPaths
addpath(genpath([pathToM '/at_utils_m']));

% for gp 
huskyDynamicsPath = pwd;
cd([pathToM '/gpml-matlab-v3.6-2015-07-07']);
run('startup');
cd(huskyDynamicsPath);

% for gp models
utilsPath = [pwd '/../utils'];
addpath([utilsPath '/models/gp']);

% for various helpers
addpath(genpath([pathToM '/neato_utils']));

% generic data processors
addpath([huskyDynamicsPath '/data_processors']);

% models
addpath(genpath([huskyDynamicsPath '/models']));

% unsorted
addpath(genpath([huskyDynamicsPath '/unsorted']));
