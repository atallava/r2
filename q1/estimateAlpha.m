% load samples
fname = 'data/samples_052120';
load(fname,'X','N','C','alpha');

% specify k(n)
k = @(n) n;

% calculate difference of sample means
NClip = calcMaxTermsForAlphaEstimate(k,N);

sampleMeans = cumsum(X)./[1:N]';
idsDiffDown = 1:NClip;
idsDiffUp = (1:NClip)+k(1:NClip);
sampleMeansDiff = sampleMeans(idsDiffUp)-sampleMeans(idsDiffDown);

%% probe sequences
gammaArray = linspace(0.05,0.25,10)';
scalingMatrix = repmat(1:NClip,length(gammaArray),1);
scalingMatrix = bsxfun(@power,scalingMatrix,gammaArray);
probeSequences = repmat(sampleMeansDiff',length(gammaArray),1).*...
    scalingMatrix;

%% plot
plot(1:NClip,probeSequences);
legendEntries = strsplit(num2str(gammaArray'));
legend(legendEntries);
