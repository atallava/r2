% if integrating with ode over short period of time
% is it enough to supply constant derivatives?

% physical params
physicalParamsFname = '../../data/traj/physical_params';
load(physicalParamsFname,'physicalParams');

% controller
controllerType = 'lqr';

switch controllerType
    case 'lqr'
        % lqr
        fnameController = '../../data/traj/lqr_tracking_controller';
        load(fnameController,'controller');

    case 'constant'
        % constant
        controller = createConstantController(0);

    otherwise
end

% init condns
stateInit = [deg2rad(20) 0];
dt = 0.01;
tInit = 0;
tFinal = tInit+dt;

% derivative as function of state
derivatives1 = @(t,x) systemDerivatives(t,x,controller(tInit,stateInit),physicalParams);
[~,odeOut1] = ode45(derivatives1,[tInit tFinal],stateInit);
stateFinal1 = odeOut1(end,:);

% derivative constant
stateDot = systemDerivatives(tInit,flipVecToColumn(stateInit),...
    controller(tInit,stateInit),physicalParams);
derivatives2 = @(t,x) flipVecToColumn(stateDot);
[~,odeOut2] = ode45(derivatives2,[tInit tFinal],stateInit);
stateFinal2 = odeOut2(end,:);

