% physical parameters
load('../data/physical_params','physicalParams');

% linearization terms
load('../data/linearization_terms','stateDesired','controlDesired',...
    'A','B','dt','Q','R');

% time steps
T = 5e3;

%% solve lqr
[K,V] = lqrControls(A,B,T,Q,R);
KFinal = K{1};

% tracking controller
controller = lqrTrackingController(KFinal,stateDesired,controlDesired);

%% test convergence
hf = figure;
KMat = cell2mat(K); % [1,T*2]
KMat = reshape(KMat,2,numel(KMat)/2);
KMat = KMat'; % [T,2]
plot(KMat);
text(T,KMat(end,1),'t = T');

%% forward simulate for some time
% close to desired
state0 = [pi+deg2rad(100) 0];

nSteps = 1e4;
[t,states,controls] = forwardSimulateODE(state0,controller,dt,nSteps,physicalParams);

%% plot
hf = vizStates([state0; states],physicalParams);