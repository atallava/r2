% load variables
gpLinVelFName = 'gp_lin_vel_01281557';
load(gpLinVelFName,'gpLinVel');
gpAngVelFName = 'gp_ang_vel_01281605';
load(gpAngVelFName,'gpAngVel');
trajFName = '../../data/dataset_1_traj_snippet.mat';
load(trajFName,'stateInit','controls','dt','states');

%% subsample traj
% even further
ids = 1:3;
states = states([ids ids(end)+1],:);
controls = controls(ids,:);
dt = dt(ids);
stateInit = states(1,:);

%% predict 
fprintf('Predicting...\n');
clockLocal = tic();
[statesPred,linVelPred,angVelPred] = predTraj(gpLinVel,gpAngVel,...
    stateInit,controls,dt);
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

%% chained 1-step predict
fprintf('Predicting...\n');
clockLocal = tic();
nMeasurements = length(dt);
statesPred = states;
[linVelPred,angVelPred] = deal(zeros(1,nMeasurements));
for i = 1:nMeasurements
    [var,linVelPred(i),angVelPred(i)] = predTraj(gpLinVel,gpAngVel,...
        states(i,:),controls(i,:),dt(i));
    statesPred(i+1,:) = var(2,:);
end
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

%% viz
figure; hold on;
plot(states(:,1),states(:,2),'b-+');
plot(statesPred(:,1),statesPred(:,2),'r-+');
axis equal;
legend('measured path','predicted path');