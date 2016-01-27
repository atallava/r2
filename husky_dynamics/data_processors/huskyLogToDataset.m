% specify file
huskyLogName = '../data/rtkposelog-2015.11.17-14.16.40-sensors.basil.mat';
datasetFName = '../data/dataset_1';

%%
logStruct = parseHuskyLog(huskyLogName);
[state,controls,t] = varsFromHuskyLogOut(logStruct);
period = min(diff(t));
dataset = datasetFromHuskyVars(state,controls,t,period);
sourceFname = huskyLogName;

%%
save(datasetFName,'sourceFname','dataset');