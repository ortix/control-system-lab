%%% Rotational Pendulum
clear;clc;
%% Open up the system
model = 'visualize';
load_system(model);

%% Define extra parameters for model
par.initial_state = [pi 0]; % Pendulum is vertical (up up)
par.torque = 0; % Input torque
par.sim_time = 100;
par.torque_bypass = 0; % Set to 0 to filter torque
Ts = 1/100;
par.Ts = Ts;    
par.h = Ts;
% Controller parameters
par.Kp = 0 ;
par.Kd = 0;
par.Ki = 0;
%% Initialize parameters and states
hws = get_param(bdroot, 'modelworkspace');

% Load model parameters into workspace from m file as a struct
hws.DataSource = 'Matlab File';
hws.FileName = 'par_struct.m';
hws.reload;

% Load additional parameters into workspace
fields = fieldnames(par);
for i = 1:numel(fields)
  hws.assignin(fields{i},par.(fields{i}));
end

% %% Linear model
% load_system('linear_model');
% hws = get_param(bdroot, 'modelworkspace');
% hws.assignin('A',sys.A);
% hws.assignin('B',sys.B);
% hws.assignin('C',sys.C);
% hws.assignin('D',sys.D);
% 

%% Linearization
model = 'visualize';

% Specify the analysis I/Os
% Specify block name as the analysis I/Os
% to linearize the block visualize/Simulation
io = 'visualize/Simulation';

% Specify the operating point
% Use the model initial condition
op = operpoint(model);


% Linearize the model
sys_cont = linearize(model,io,op)

% Discretize the model
sys = c2d(sys_cont,Ts,'zoh')
% sys=sys_cont

%% LQR Pole Placement
% Find the poles
clf
poles = eig(sys.A)

% Determine controlability
co = ctrb(sys);
controllability = rank(co)
% LQR Designsys
Q = sys.C'*sys.C

%%%%% Optimize this %%%%%%%%%%
% Q(2,2) = 1; %theta1
% Q(3,3) = 1; %theta2
R = 0.001;
[K,~,e] = dlqr(sys.A,sys.B,Q,R)

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

%Precompensator
Nbar = 1;

sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);
impulse(sys_cl)

%% Observer
observability = rank(obsv(sys))

% Check stability of system with state feedback
poles = eig(sys_cl)
slowest = real(max(poles))/5;
% Place the poles
P = [slowest slowest+0.01 slowest+0.02 slowest+0.03 slowest+0.04];
L = place(sys.A',sys.C',P)'

% Controller
Ace = [(sys.A-sys.B*K) (sys.B*K); zeros(size(sys.A)) (sys.A-L*sys.C)];
Bce = [sys.B; zeros(size(sys.B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states = {'T_dot' 'theta1' 'theta2' 'theta1_dot' 'theta2_dot' 'e1' 'e2' 'e3' 'e4' 'e5'};
inputs = {'torque'};
outputs = {'theta1'; 'theta2'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,Ts,'statename',states,'inputname',inputs,'outputname',outputs);
hold on
impulse(sys_est_cl)

%% Save the state matrices
state.A = sys.a;
state.B = sys.b;
state.C = sys.c;
state.D = sys.d;
state.K = K;
state.L = L;
state.h = Ts

%% Save the params to the model
addpath('plant');
open_system('rpend');
simh = get_param(bdroot, 'modelworkspace');

fields = fieldnames(state);
for i = 1:numel(fields)
  simh.assignin(fields{i},state.(fields{i}));
end
