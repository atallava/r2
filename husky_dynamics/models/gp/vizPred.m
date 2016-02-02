% load variables
gpLinVelFName = 'gp_lin_vel_01281557';
load(gpLinVelFName,'gpLinVel');
gpAngVelFName = 'gp_ang_vel_01281605';
load(gpAngVelFName,'gpAngVel');
datasetFName = '../../data/dataset_gp_1_train_subsampled';
load(datasetFName);

%% subsample data
nElements = size(x,1);
ids = randsample(1:nElements,100);
x = x(ids,:); y = y(ids,:);

%% predict
yPred = zeros(size(y));
yPred(:,1) = gpLinVel(x);
yPred(:,2) = gpAngVel(x);

% get states
nElements = size(x,1);
statesInit = x(:,1:3);
dt = 0.01;
statesFinal = zeros(nElements,3);
statesFinalPred = zeros(nElements,3);
for i = 1:nElements
    statesFinal(i,3) = statesInit(i,3)+y(i,2)*dt;
    statesFinal(i,1) = statesInit(i,1)+y(i,1)*dt*cos(statesFinal(i,3));
    statesFinal(i,2) = statesInit(i,2)+y(i,1)*dt*sin(statesFinal(i,3));
    
    statesFinalPred(i,3) = statesInit(i,3)+yPred(i,2)*dt;
    statesFinalPred(i,1) = statesInit(i,1)+yPred(i,1)*dt*cos(statesFinalPred(i,3));
    statesFinalPred(i,2) = statesInit(i,2)+yPred(i,1)*dt*sin(statesFinalPred(i,3));
end

%% viz
hf = figure; 
axis equal; hold on;

% repackage variables for plot
[u,v,p,q] = deal(zeros(2,nElements));
u(1,:) = statesInit(:,1); u(2,:) = statesFinal(:,1);
v(1,:) = statesInit(:,2); v(2,:) = statesFinal(:,2);
p(1,:) = statesInit(:,1); p(2,:) = statesFinalPred(:,1); 
q(1,:) = statesInit(:,2); q(2,:) = statesFinalPred(:,2);
%%
plot(u,v,'b-+'); 
quiver(statesInit(:,1),statesInit(:,2),cos(statesInit(:,3)),sin(statesInit(:,3)),...
    0.005,'color',[0 0 1],'showarrowhead','off');
quiver(statesFinal(:,1),statesFinal(:,2),cos(statesFinal(:,3)),sin(statesFinal(:,3)),...
    0.005,'color',[0 0 1],'showarrowhead','off');
plot(p,q,'r--+');
quiver(statesFinalPred(:,1),statesFinalPred(:,2),cos(statesFinalPred(:,3)),sin(statesFinalPred(:,3)),...
    0.005,'color',[1 0 0],'showarrowhead','off');

legend('meas','pred');
