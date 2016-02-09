function [stateFinal,stateArray] = fwdSim(stateInit,controls,dt,W)
  % controls are (vl,vr)
  % W is wheelbase
  nControls = size(controls,1);
  VArray = sum(controls,2)*0.5;
  wArray = (controls(:,2)-controls(:,1))/W;
  stateArray = zeros(nControls+1,3);
  stateArray(1,:) = stateInit;
  dTh = wArray.*dt;
  ds = VArray.*dt;
  for i = 1:nControls
    stateArray(i+1,3) = stateArray(i,3)+dTh(i)
    stateArray(i+1,1) = stateArray(i,1)+cos(stateArray(i+1,3))*ds(i);
    stateArray(i+1,2) = stateArray(i,2)+sin(stateArray(i+1,3))*ds(i);
  end
end
