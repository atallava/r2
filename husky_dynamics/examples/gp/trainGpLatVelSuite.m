% common struct fields
trainStructCommon.fnameTrain = '../../data/dataset_gp_lat_vel_1_train_subsampled';
trainStructCommon.infMethod = @infExact;
trainStructCommon.likFunc = @likGauss;
trainStructCommon.maxIterations = 50;
trainStructCommon.gpWrapperName = 'gpLatVel';

testStructCommon.gpWrapperName = 'gpLatVel';
testStructCommon.datasetFname = '../../data/dataset_gp_lat_vel_1_hold.mat';
testStructCommon.vizFlag = 0;

%% specify param set
dimState = 5;
sf = 2; % TODO: what does this mean?
vecArd = rand(dimState,1);

meanFuncs = {};
covFuncs = {};
hyps = struct('mean',{},'cov',{},'lik',{});

% first, keeping cov const, different means
meanFuncs = { ...
    {@meanConst},{@meanLinear},{@meanPoly,2},{@meanPoly,3} ...
    };
covFuncs(1:4) = {@covSEard};

[hyps(1:4).cov] = deal(log([vecArd; sf]));
[hyps(1:4).lik] = deal(log(0.1));
hyps(1).mean = 0;
hyps(2).mean = zeros(dimState,1);
hyps(3).mean = zeros(2*dimState,1);
hyps(4).mean = zeros(3*dimState,1);

% next, keeping mean const, different covs
meanFuncs(5:8) = {{@meanConst}};
covFuncs = [covFuncs ...
    {'covLINard'},{{@covMaternard,5}},{@covNNone},{{@covSEfact,2}} ...
    ];
[hyps(5:8).mean] = deal(0);
[hyps(5:8).lik] = deal(log(0.1));
hyps(5).cov = log(vecArd);
hyps(6).cov = log([vecArd; sf]);
hyps(7).cov = log([vecArd; sf]);
hyps(8).cov = randn(2*dimState,1);

nParams = length(meanFuncs);

%% let it rip
clockLocal = tic();
for i = 1:nParams
    fprintf('Parameter set %d.\n',i);
    
    % train
    trainStruct = trainStructCommon;
    trainStruct.gpWrapperFname = sprintf('gp_lat_vel_suite_%s_%d',myDateStamp(2:3),i);
    trainStruct.meanFunc = meanFuncs{i};
    trainStruct.covFunc = covFuncs{i};
    trainStruct.hyp = hyps(i);
    try
        trainGpFn(trainStruct);
    catch
        warning('Train failed. Continuing...');
        continue;
    end

    % test
    testStruct = testStructCommon;
    testStruct.gpWrapperFname = trainStruct.gpWrapperFname;
    testStruct.fnameOut = [testStruct.gpWrapperFname '_err'];
    testGpFn(testStruct);
end
tComp = toc(clockLocal);
