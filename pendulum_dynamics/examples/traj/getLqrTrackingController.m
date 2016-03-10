% physical parameters
fnamePhysicalParams = '../data/physical_params';
load(fnamePhysicalParams,'physicalParams');

% linearization terms
fnameLinearization = '../data/linearization_terms';
load(fnameLinearization,'stateDesired','controlDesired',...
    'A','B','dt','Q','R');

% time steps
T = 5e3;

%% solve lqr
[K,V] = solveLqr(A,B,T,Q,R);
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

%% save
fnameController = '../data/lqr_tracking_controller';
save(fnameController);