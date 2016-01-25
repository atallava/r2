fnameTrain = '../../data/dataset_gp_1_train_subsampled';
load(fnameTrain);

%% params
yang = y(:,2);
infMethod = @infExact;
meanFunc = [];
covFunc = @covSEiso; 
hyp.cov = [0; 0];
likFunc = @likGauss;
hyp.lik = log(0.1);

%% optimize hyperparams
fprintf('Training gp ang vel.\n');
clockLocal = tic();
maxIterations = 50;
hyp = minimize(hyp,@gp,-maxIterations,@infExact,meanFunc,covFunc,likFunc,x,yang);
tComp = toc(clockLocal);
fprintf('Computation time: %.3f.\n',tComp);
nll = gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,yang);
fprintf('Training data nll: %.3f.\n',nll);

%% create gp ang vel
gpAngVel = @(xQuery) gp(hyp,infMethod,meanFunc,covFunc,likFunc,...
    x,yang,xQuery);

%% save
fname = ['gp_ang_vel' '_' myDateStamp(2:5)] ;
save(fname);
