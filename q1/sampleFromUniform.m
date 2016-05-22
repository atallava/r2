function X = sampleFromUniform(distribParamsStruct,nSamples)
if nargin < 2
    nSamples = 1;
end
   
lb = distribParamsStruct.mean-distribParamsStruct.range*0.5;
X = rand(nSamples,1)*distribParamsStruct.range+lb;
end