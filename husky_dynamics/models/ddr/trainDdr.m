function WOptim = trainDdr(fnameTrain)
  load(fnameTrain);

  fun = @(W) predictionRisk(dataset.statesFinal,predict(dataset,W));
  lb = 1e-6;
  ub = 5; 
  W0 = 1;
  [WOptim,objOptim,optExitflag,optOutput] = fmincon(fun,W0,[],[],[],[],lb,ub);
end
