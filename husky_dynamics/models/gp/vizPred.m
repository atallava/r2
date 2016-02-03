% load variables
gpLinVelFName = 'gp_lin_vel_02021657';
load(gpLinVelFName,'gpLinVel');
gpAngVelFName = 'gp_lat_vel_02021650';
load(gpAngVelFName,'gpLatVel');
gpAngVelFName = 'gp_ang_vel_02021654';
load(gpAngVelFName,'gpAngVel');

datasetFName = '../../data/dataset_gp_lin_vel_1_hold';
load(datasetFName);
linVel = y;

datasetFName = '../../data/dataset_gp_lat_vel_1_hold';
load(datasetFName);
latVel = y;

datasetFName = '../../data/dataset_gp_ang_vel_1_hold';
load(datasetFName);
angVel = y;

%% subsample data
nElements = size(x,1);
ids = randsample(1:nElements,100);
x = x(ids,:);
linVel = linVel(ids,:);
latVel = latVel(ids,:);
angVel = angVel(ids,:);

%% predict
linVelPred = gpLinVel(x);
latVelPred = gpLatVel(x);
angVelPred = gpAngVel(x);

%% calc states
nElements = size(x,1);
statesInit = x(:,1:3);
dt = 0.01*ones(nElements,1);
statesFinal = bodyVelsToState(statesInit,linVel,latVel,angVel,dt);
statesFinalPred = bodyVelsToState(statesInit,linVelPred,latVelPred,angVelPred,dt);

%% state err
posnErr = statesFinal(:,1:2)-statesFinalPred(:,1:2);
posnErr = posnErr.^2; posnErrVec = sqrt(sum(posnErr,2)); 
thErrVec = thDiff(statesFinal(:,3),statesFinalPred(:,3));
fprintf('posn err. Mean: %.4f, std: %.4f\n',mean(posnErrVec),std(posnErrVec));
fprintf('th err. Mean: %.4f, std: %.4f\n',mean(thErrVec),std(thErrVec));

%% viz
hf = figure; 
axis equal; hold on;

% repackage variables for plot
[u,v,p,q] = deal(zeros(2,nElements));
u(1,:) = statesInit(:,1); u(2,:) = statesFinal(:,1);
v(1,:) = statesInit(:,2); v(2,:) = statesFinal(:,2);
p(1,:) = statesInit(:,1); p(2,:) = statesFinalPred(:,1); 
q(1,:) = statesInit(:,2); q(2,:) = statesFinalPred(:,2);

plot(u,v,'b-+'); 
quiver(statesInit(:,1),statesInit(:,2),cos(statesInit(:,3)),sin(statesInit(:,3)),...
    0.005,'color',[0 0 1],'showarrowhead','off');
quiver(statesFinal(:,1),statesFinal(:,2),cos(statesFinal(:,3)),sin(statesFinal(:,3)),...
    0.005,'color',[0 0 1],'showarrowhead','off');
plot(p,q,'r--+');
quiver(statesFinalPred(:,1),statesFinalPred(:,2),cos(statesFinalPred(:,3)),sin(statesFinalPred(:,3)),...
    0.005,'color',[1 0 0],'showarrowhead','off');

legend('meas','pred');
