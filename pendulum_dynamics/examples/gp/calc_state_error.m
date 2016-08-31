% load
% dynamics dataset
dynDatasetFname = '../../data/dyn/dataset_dyn_1_hold.mat';
load(dynDatasetFname,'dataset','physicalParams');

% predictor 
predictorFname = '../../data/gp/gp_predictor_03101431';
load(predictorFname,'gpPredictor');

%% subsample
ids = randsample(1:length(dataset),35);
dataset = dataset(ids);

%% 
% predictor struct
predictorStruct.predictor = gpPredictor;
predictorStruct.dynStateToPredictorState = @transformDynStateToGpState;

nElements = length(dataset);

stateInits = [dataset.stateInit];
stateInits = reshape(stateInits,numel(stateInits)/2,2);
stateFinals = [dataset.stateFinal];
stateFinals = reshape(stateFinals,numel(stateFinals)/2,2);
stateFinalsPred = zeros(size(stateFinals));

clockLocal = tic();
for i = 1:nElements
    stateInit = dataset(i).stateInit;
    stateFinal = dataset(i).stateFinal;
    % ASSUMING: 1-step predictions
    dt = dataset(i).dt;
    
    [t,statesPred,statesDotPred] = forwardSimulateODEWithPredictor(stateInit,dt,1,predictorStruct);
    stateFinalsPred(i,:) = statesPred(end,:);
end
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

stateErr = norm(stateFinalsPred-stateFinals);
fprintf('mean error in theta prediction: %.3f.\n',stateErr);