% check variables
condn = logical(exist('gpWrapperFname','var'));
assert(condn,'gpWrapperFname not assigned.');
condn = logical(exist('gpWrapperName','var'));
assert(condn,'gpWrapperName not assigned.');
condn = logical(exist('datasetFname','var'));
assert(condn,'datasetFname not assigned.');
condn = logical(exist('fnameOut','var'));
assert(condn,'fnameOut not assigned.');
condn = logical(exist('vizFlag','var'));
assert(condn,'vizFlag not assigned.');

%% load
tmp = load(gpWrapperFname,gpWrapperName);
gpWrapper = tmp.(gpWrapperName);
load(datasetFname);

%% predict
yPred = gpWrapper(x);

%% errors
errVec = (yPred-y).^2;
meanErr = mean(errVec);
stdErr = std(errVec);
fprintf('Mean error: %.3f. std: %.3f.\n',meanErr,stdErr);

%% viz
if vizFlag
    hf = figure;
    plot(y,'.'); hold on;
    plot(yPred,'.');
    legend('meas','pred');
    dcm_obj = datacursormode(hf);
    set(dcm_obj,'UpdateFcn',@(obj,eventObj) tagPlotPointWithId(obj,eventObj,1:length(yPred),yPred));
end

%% save
if ~isempty(fnameOut)
    save(fnameOut);
end