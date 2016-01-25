fnameTrain = '../../data/dataset_gp_1_train_subsampled';
load(fnameTrain);

%% params
ylin = y(:,1);
infMethod = @infExact;
meanFunc = [];
covFunc = @covSEiso; 
hyp.cov = [0; 0];
likFunc = @likGauss;
hyp.lik = log(0.1);

%% optimize hyperparams
fprintf('Training gp lin vel.\n');
clockLocal = tic();
maxIterations = 50;
hyp = minimize(hyp,@gp,-maxIterations,@infExact,meanFunc,covFunc,likFunc,x,ylin);
tComp = toc(clockLocal);
fprintf('Computation time: %.3f.\n',tComp);
nll = gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,ylin);
fprintf('Training data nll: %.3f.\n',nll);

%% create gp lin vel
gpLinVel = @(xQuery) gp(hyp,infMethod,meanFunc,covFunc,likFunc,...
    x,ylin,xQuery);

%% save
fname = ['gp_lin_vel' '_' myDateStamp(2:5)] ;
save(fname);


