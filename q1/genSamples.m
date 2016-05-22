% mean sequence parameters
C = 15;
alpha = 0.25;

% target mean
meanTarget = 10; % TODO: is the convention nounAdjective or adjectiveNoun?

% distribution
sampleFn = @sampleFromUniform;
uniformDistRange = 2;

% number of samples
N = 1000;

% generate samples
X = zeros(N,1);
distribParamsStructCell = cell(N,1);
for n = 1:N
    distribParamsStruct.mean = calcSequenceTerm(C,alpha,meanTarget,n);
    distribParamsStruct.range = uniformDistRange;
    X(n) = sampleFromUniform(distribParamsStruct);
    
    % log
    distribParamsStructCell{n} = distribParamsStruct;
end

%% write to file
fname = ['data/samples' '_' myDateStamp(2:4)];
save(fname,'C','alpha',...
    'sampleFn','distribParamsStructCell','N',...
    'X');