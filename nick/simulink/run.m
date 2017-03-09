%%% Rotational Pendulum
%% Open up the system
model = 'visualize';
load_system(model);

%% Define extra parameters for model
par.initial_state = [0 0]; % Pendulum is vertical (up up)
par.torque = 0; % Input torque
par.sim_time = 100;
par.torque_bypass = 1; % Set to 0 to filter torque

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
% return

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
sys_cont = linearize(model,io,op);

% Discretize the model

Ts = 1/100; 
sys = c2d(sys_cont,Ts,'zoh')
% sys=sys_cont;

%% LQR Pole Placement
% Find the poles
poles = eig(sys.A);

% Determine controlability
co = ctrb(sys);
controllability = rank(co)

% LQR Designsys
Q(1,1) = 10; % theta1
Q(2,2) = 50;
Q(3,3) = 10; % theta1_d
Q(4,4) = 1;
R = 0.1;
[K,~,e] = dlqr(sys.A,sys.B,Q,R)

% % Find Nbar to eliminate steady state error
% Cn = [1 0 0 0]; % Only affect input of rotor
% sys_ss = ss(sys.A,sys.B,Cn,0);
% Nbar = rscale(sys_ss,K)

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

%% Observer
observability = rank(obsv(sys))

% Check stability of system with state feedback
poles = eig(sys)

% Place the poles
P = [-7952 -10 -11 -12];
L = place(sys.A',sys.C',P)'

% Controller
Ace = [(sys.A-sys.B*K) (sys.B*K); zeros(size(sys.A)) (sys.A-L*sys.C)];
Bce = [sys.B; zeros(size(sys.B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states = {'theta1' 'theta2' 'theta1_dot' 'theta2_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'torque'};
outputs = {'theta1'; 'theta2'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,Ts,'statename',states,'inputname',inputs,'outputname',outputs);
hold on
impulse(sys_est_cl)
