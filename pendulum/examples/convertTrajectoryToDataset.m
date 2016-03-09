fnameTrajectory = '../data/uncontrolled_pendulum_trajectory';
load(fnameTrajectory,'physicalParams','controller','dt','state0','nSteps',...
    't','states','controls');

%% check accelerations
nStates = size(states,1);
actualAccn = zeros(nStates,1);
finiteDiffAccn = zeros(nStates,1);

for i = 1:nStates
    if i == 1
        state = state0;
    else
        state = states(i-1,:);
    end
    xDot = systemDerivatives(t(i)-dt,state',controls(i),physicalParams);
    actualAccn(i) = xDot(2);
end
finiteDiffAccn(2:end) = diff(states(:,2))/dt;
finiteDiffAccn(1) = (states(1,2)-state0(2))/dt;