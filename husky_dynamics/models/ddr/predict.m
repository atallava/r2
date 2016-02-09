function statesFinal = predict(dataset,W)
  nElements = length(dataset);
  statesFinal = zeros(nElements,3);
  for i = 1:nElements
    stateInit = dataset(i).stateInit;
    controls = dataset(i).controls;
    dt = dataset(i).dt;
    statesFinal(i,:) = fwdSim(stateInit,controls,dt,W);
  end
end
