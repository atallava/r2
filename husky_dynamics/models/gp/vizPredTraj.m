% load variables
gpLinVelFName = 'gp_lin_vel_01251736';
load(gpLinVelFName,'gpLinVel');
gpAngVelFName = 'gp_ang_vel_01251740';
load(gpAngVelFName,'gpAngVel');
trajFName = '../../data/trajectory_snippet.mat';
load(trajFName,'states','controls','dt');
stateInit = states(1,:);

%% predict
fprintf('Predicting...\n');
clockLocal = tic();
[statesPred,linVelPred,angVelPred] = predTraj(gpLinVel,gpAngVel,...
    stateInit,controls,dt);
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

%% viz
figure; hold on;
plot(states(:,1),states(:,2),'b');
plot(statesPred(:,1),statesPred(:,2),'r');
axis equal;
legend('measured path','predicted path');