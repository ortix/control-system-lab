%%% Rotational Pendulum
clear;clc;
%% Open up the system
model = 'visualize';
load_system(model);

%% Define extra parameters for model
params.initial_state = [0 0]; % Pendulum is vertical (up up)
params.torque = 0; % Input torque
params.sim_time = 10;
params.torque_bypass = 0; % Set to 0 to filter torque
Ts = 1/500;
params.Ts = Ts;    
params.h = Ts;
% Controller parameters
params.Kp = 0 ;
params.Kd = 0;
params.Ki = 0;
%% Initialize parameters and states
hws = get_param(bdroot, 'modelworkspace');

% Load model parameters into workspace from m file as a struct
hws.DataSource = 'Matlab File';
hws.FileName = 'par_struct.m';
hws.reload;

toModelWorkspace('visualize',params);
toModelWorkspace('model_v2',params);

% %% Linear model
% load_system('linear_model');
% hws = get_param(bdroot, 'modelworkspace');
% hws.assignin('A',sys.A);
% hws.assignin('B',sys.B);
% hws.assignin('C',sys.C);
% hws.assignin('D',sys.D);
% 

%% Linearization
model = 'model_v2';

% Specify the operating point
% Use the model initial condition
op = operpoint(model);

% Create the analysis I/O variable IOs1
io(1) = linio('model_v2/Torque_in',1,'input');
io(2) = linio('model_v2/Demux',1,'output');
io(3) = linio('model_v2/Demux',2,'output');

% Linearize the model
sys_cont = linearize(model,op,io)

% Discretize the model
sys_disc = c2d(sys_cont,Ts,'zoh');

% Analytical linearization
sys_analytical = getLinearModel(params.initial_state)
sys_anl_disc = c2d(sys_analytical,Ts,'zoh');

% Pick the system we want to use
sys = sys_cont;

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
Q = diag([1 1 8000 0 0.5])
R = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[K,~,e] = lqr(sys.A,sys.B,Q,R)

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

sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
impulse(sys_cl)

%% Observer
observability = rank(obsv(sys))
sys
% Check stability of system with state feedback
poles = eig(sys_cl)
lambda = real(max(poles))/10;

% Place the poles
P = [lambda lambda-1 lambda-2 lambda-3 lambda-4];
L = place(sys.A.',sys.C.',P).'

% Controller
Ace = [(sys.A-sys.B*K) (sys.B*K); zeros(size(sys.A)) (sys.A-L*sys.C)];
Bce = [sys.B; zeros(size(sys.B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states = {'theta1' 'theta2' 'theta1_dot' 'theta2_dot' 'T_dot' 'e1' 'e2' 'e3' 'e4' 'e5'};
inputs = {'torque'};
outputs = {'theta1'; 'theta2'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);

%% Simulate
% t = 0:Ts:5;
% r = 0.1*ones(size(t));
% [y,t,x]=lsim(sys_est_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','Torque')
% set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% title('Step Response with Observer-Based State-Feedback Control')
% grid on
%% Save the state matrices
state.A = sys.a;
state.B = sys.b;
state.C = sys.c;
state.D = sys.d;
state.K = K;
state.L = L;
state.h = Ts

toModelWorkspace('visualize',state);

% %% Save the params to the model
% addpath('plant');
% open_system('rpend');
% simh = get_param(bdroot, 'modelworkspace');
% 
% fields = fieldnames(state);
% for i = 1:numel(fields)
%   simh.assignin(fields{i},state.(fields{i}));
% end