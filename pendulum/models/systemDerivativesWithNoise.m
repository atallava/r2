function xDot = systemDerivativesWithNoise(t,x,controls,physicalParams)
    % t has to be scalar
    % x has to be column vector
    
    xDot = zeros(size(x));
    xDot(1) = x(2);
    xDot(2) = (controls-physicalParams.m*physicalParams.g*physicalParams.l*sin(x(1)))/...
        (physicalParams.m*physicalParams.l^2);
    xDot = xDot+normrnd([0 0],physicalParams.noiseVariance);
end