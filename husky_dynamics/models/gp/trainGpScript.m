% check variables
condn = exist('fnameTrain','var'); condn = logical(condn);
assert(condn,sprintf('Input fnameTrain does not exist.'));
condn = exist('gpWrapperFname','var'); condn = logical(condn);
assert(condn,sprintf('Input gpWrapperFname does not exist.'));
condn = exist('infMethod','var'); condn = logical(condn);
assert(condn,sprintf('Input infMethod does not exist.'));
condn = exist('meanFunc','var'); condn = logical(condn);
assert(condn,sprintf('Input meanFunc does not exist.'));
condn = exist('covFunc','var'); condn = logical(condn);
assert(condn,sprintf('Input covFunc does not exist.'));
condn = exist('hyp','var'); condn = logical(condn);
assert(condn,sprintf('Input hyp does not exist.'));
condn = exist('likFunc','var'); condn = logical(condn);
assert(condn,sprintf('Input likFunc does not exist.'));
condn = exist('maxIterations','var'); condn = logical(condn);
assert(condn,sprintf('Input maxIterations does not exist.'));
condn = exist('gpWrapperName','var'); condn = logical(condn);
assert(condn,sprintf('Input gpWrapperName does not exist.'));

%% train dataset
load(fnameTrain);

%% optimize hyperparams
fprintf('Training %s.\n',gpWrapperName);
clockLocal = tic();

% optimize on training nll
fun = @(hyp) gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y);

hyp = minimize(hyp,fun,-maxIterations);
tComp = toc(clockLocal);
fprintf('Computation time: %.3f.\n',tComp);
nll = gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y);
fprintf('Training data nll: %.3f.\n',nll);

%% create gp wrapper function
line = sprintf('%s = @(xQuery) gp(hyp,infMethod,meanFunc,covFunc,likFunc,x,y,xQuery);',gpWrapperName);
eval(line);

%% save
save(gpWrapperFname);