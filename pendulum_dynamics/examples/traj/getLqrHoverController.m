% physical parameters
fnamePhysicalParams = '../../data/traj/physical_params';
load(fnamePhysicalParams,'physicalParams');

% linearization terms
fnameLinearization = '../../data/traj/linearization_terms';
load(fnameLinearization,'stateDesired','controlDesired',...
    'A','B','dt','Q','R');

% time steps
T = 5e3;

%% solve lqr
[K,V] = solveLqr(A,B,T,Q,R);
KFinal = K{1};

% tracking controller
controller = @(t,states) calcLqrHoverControls(t,states,KFinal,stateDesired,controlDesired);

%% test convergence
hf = figure;
KMat = cell2mat(K); % [1,T*2]
KMat = reshape(KMat,2,numel(KMat)/2);
KMat = KMat'; % [T,2]
plot(KMat);
text(T,KMat(end,1),'t = T');

%% save
fnameController = '../../data/traj/lqr_hover_controller';
save(fnameController);