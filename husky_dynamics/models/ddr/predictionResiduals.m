function residuals = predictionResiduals(W,dataset)
  nElements = length(dataset);
  residuals = zeros(nElements,3);

  statesFinalPred = predict(W,dataset);
  % TODO: check this access method
  statesFinal = [dataset.stateFinal];
  residuals(:,1:2) = stateFinal(:,1:2)-stateFinalPred(:,1:2);
  residuals(:,3) = thDiff(stateFinalPred(:,3),stateFinal(:,3));
end
