%%% Rotational Pendulum
clear;clc;
%% Open up the system
model = 'visualize';
load_system(model);
%% Define extra parameters for model
disp('Initializing parameters');
params.initial_state = [0 0]; % Pendulum is vertical (up up)
params.torque = 0; % Input torque
params.sim_time = 10;
params.torque_bypass = 0; % Set to 0 to filter torque
Ts = 1/100;
params.Ts = Ts;    
params.h = Ts;
%% Initialize parameters and states
disp('Writing parameters to model workspaces');
% Load states into model
toModelWorkspace('visualize',params);
toModelWorkspace('model_v2',params);

% Load parameters into model
data = load('params.mat');
toModelWorkspace('visualize',data);
toModelWorkspace('model_v2',data);

%% Linearization
disp('Linearizing and discretizing model');
model = 'model_v2';

% Specify the operating point
% Use the model initial condition
op = operpoint(model);

% Create the analysis I/O variable IOs1
io(1) = linio('model_v2/Torque_in',1,'input');
io(2) = linio('model_v2/Demux',1,'output');
io(3) = linio('model_v2/Demux',2,'output');

% Linearize the model
sys_cont = linearize(model,op,io);

% Discretize the model
sys_disc = c2d(sys_cont,Ts,'zoh');

% Pick the system we want to use
sys = sys_disc;

%% LQR Pole Placement
disp('Checking controllability');
% Find the poles
clf
poles = eig(sys.A);

% Determine controlability
co = ctrb(sys);
controllability = rank(co);
if(controllability == length(sys.b))
   disp('System is controllable') 
else
    error('System is not controllable')
end
% LQR Designsys
Q = sys.C'*sys.C;

%%%%% Optimize this %%%%%%%%%%
Q = diag([1 10000 100000 100 1]);
R = 50000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Calculating LQR poles');
[K,~,e] = dlqr(sys.A,sys.B,Q,R);

% Create new state space representation with full state feedback by
% using K found with LQR
Ac = [(sys.A-sys.B*K)];
Bc = [sys.B];
Cc = [sys.C];
Dc = [sys.D];

% Plot our results
states = sys.StateName;
inputs = {'torque'};
outputs = {'theta1'; 'theta2'};

sys_cl = ss(Ac,Bc,Cc,Dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);
% impulse(sys_cl)
grid on
%% Save the state matrices
state.A = sys.a;
state.B = sys.b;
state.C = sys.c;
state.D = sys.d;
state.K = K;
state.h = Ts;
state.Ts = Ts;
state.initial_state = params.initial_state;
state.Q = Q;
state.R = R;
toModelWorkspace('visualize',state);
addpath('plant')
toModelWorkspace('rpend',state);

%% Simulate
disp('Running simulation...');
evalc('sim(''visualize'')');
disp('Simulation complete!');

%% Plot graphs
disp('Creating pretty graphs')
% Theta1
subplot(3,1,1);hold on;grid on;
plot(yhat.time,yhat.Data(:,1))
plot(yhat.time,yhat.Data(:,3),'LineWidth',1)
title('Theta 1 angle')
ylabel('amplitude [rad]');xlabel('time [s]')

%Theta2
subplot(3,1,2);hold on; grid on;
plot(yhat.time,yhat.Data(:,2))
plot(yhat.time,yhat.Data(:,4),'LineWidth',1)
title('Theta 2 angle')
ylabel('amplitude [rad]');xlabel('time [s]')

% Disturbance
subplot(3,1,3);hold on; grid on;
plot(disturbance.time,disturbance.Data(:,2),'LineWidth',1)
title('Disturbance')
ylabel('amplitude [rad]');xlabel('time [s]')
saveas(gcf,'../presentation/img/disturbance_effect','png');

% Torque
figure;
plot(torque,'linewidth',0.7);
grid on;
ylim([-1 1])
saveas(gcf,'../presentation/img/torque','png');

disp('I can haz good grade?')


