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
R = 90000;
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
impulse(sys_cl)
grid on

%% Observer
Kw_1 = 0.01;
Kw_2 = 0.01;
alpha = 2;
zeta = 2;
t_set = 0.2;
h = Ts;
w = 4.6/(zeta*t_set)*alpha;       % settling time of 8 seconds
p1 = -2*exp(-2*zeta*w*h)*cos(w*h*sqrt(1-zeta^2));
p2 = exp(-2*zeta*w*h);
syms s
b = s^2 + p1*s + p2;
lambda = solve(b,s);
lambda = single(lambda);
observer_poles = [real(lambda(1)) real(lambda(2)) real(lambda(1))^alpha -0.01+real(lambda(1))^alpha 0.01+real(lambda(1))^alpha];
L = place(sys_disc.a',sys_disc.c',observer_poles)';
closedloop = sys_disc.a - L*sys_disc.c;
eig(closedloop)
return
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
% toModelWorkspace('rpend',state);
toModelWorkspace('rpend_observer',state)
