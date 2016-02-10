function [risk,losses] = statePredictionRisk(states,statesPred)
  scale = 0.5/0.44;
  mat = states-statesPred;
  mat = mat.^2;
  mat(:,3) = mat(:,3)*scale;
  mat = sum(mat,2);
  losses = sqrt(mat);
  risk = mean(losses);
end
