function statesFinal = predict(W,dataset)
  nElements = length(dataset);
  statesFinal = zeros(nElements,3);
  for i = 1:nElements
    stateInit = dataset(i).stateInit;
    controls = dataset(i).controls;
    dt = dataset(i).dt;
    statesFinal(i,:) = fwdSim(W,stateInit,controls,dt);
  end
end
