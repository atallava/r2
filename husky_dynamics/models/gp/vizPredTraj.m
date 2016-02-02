% load variables
gpLinVelFName = 'gp_lin_vel_02021657';
load(gpLinVelFName,'gpLinVel');
gpAngVelFName = 'gp_lat_vel_02021650';
load(gpAngVelFName,'gpLatVel');
gpAngVelFName = 'gp_ang_vel_02021654';
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
[statesPred,linVelPred,latVelPred,angVelPred] = predTraj(gpLinVel,gpLatVel,gpAngVel,...
    stateInit,controls,dt);
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

%% chained 1-step predict
fprintf('Predicting...\n');
clockLocal = tic();
nMeasurements = length(dt);
statesPred = states;
[linVelPred,latVelPred,angVelPred] = deal(zeros(1,nMeasurements));
for i = 1:nMeasurements
    [var,linVelPred(i),latVelPred(i),angVelPred(i)] = predTraj(gpLinVel,gpLatVel,gpAngVel,...
        states(i,:),controls(i,:),dt(i));
    statesPred(i+1,:) = var(2,:);
end
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

%% viz
quiverScale = 0.1;

figure; hold on;
plot(states(:,1),states(:,2),'b-+');
quiver(states(:,1),states(:,2),cos(states(:,3)),sin(states(:,3)),...
    quiverScale,'color',[0 0 1],'showarrowhead','off');
plot(statesPred(:,1),statesPred(:,2),'r-+');
quiver(statesPred(:,1),statesPred(:,2),cos(statesPred(:,3)),sin(statesPred(:,3)),...
    quiverScale,'color',[1 0 0],'showarrowhead','off');
axis equal;
