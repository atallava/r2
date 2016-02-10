% load variables
modelFname = 'data/ddr_model_1';
load(modelFname,'W');

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
[stateFinal,statesPred] = fwdSim(W,stateInit,controls,dt);
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

%% chained 1-step predict
fprintf('Predicting...\n');
clockLocal = tic();
nMeasurements = length(dt);
statesPred = states;
for i = 1:nMeasurements
    [~,stateArray] = fwdSim(W,states(i,:),controls(i,:),dt(i));
    statesPred(i+1,:) = stateArray(end,:);
end
tComp = toc(clockLocal);
fprintf('Computation time: %.3fs.\n',tComp);

%% viz
quiverScale = 5e-2;

figure; 
hold on; 
axis equal;
plot(states(:,1),states(:,2),'b-');
quiver(states(:,1),states(:,2),quiverScale*cos(states(:,3)),quiverScale*sin(states(:,3)),...
    'color',[0 0 1],'showarrowhead','on','autoscale','off');
plot(statesPred(:,1),statesPred(:,2),'r-');
quiver(statesPred(:,1),statesPred(:,2),quiverScale*cos(statesPred(:,3)),quiverScale*sin(statesPred(:,3)),...
    'color',[1 0 0],'showarrowhead','on','autoscale','off');
