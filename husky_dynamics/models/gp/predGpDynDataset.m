function statesFinal = predGpDynDataset(dataset,gpLinVel,gpLatVel,gpAngVel)
    nElements = length(dataset);
    x = zeros(nElements,5);
    % some tricks to get exact sized arrays
    statesInit = [dataset.stateInit];
    statesInit = reshape(statesInit,3,numel(statesInit)/3);
    statesInit = statesInit'; % [nStates,3]
    
    controls = [dataset.controls];
    controls = reshape(controls,2,numel(controls)/2);
    controls = controls'; % [nStates,2]
    
    x = dynStatesToGpStates(statesInit,controls); % [nStates,5]
   
    linVelPred = gpLinVel(x);
    latVelPred = gpLatVel(x);
    angVelPred = gpAngVel(x);
    statesFinal = bodyVelsToState(statesInit,linVelPred,latVelPred,angVelPred,dt);
end