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

%% Linearization
model = 'visualize';

%% Specify the analysis I/Os
% Specify block name as the analysis I/Os
% to linearize the block visualize/Simulation
io = 'visualize/Simulation';

%% Specify the operating point
% Use the model initial condition
op = operpoint(model);


%% Linearize the model
sys = linearize(model,io,op);

%% Analysis
% Find the poles
poles = eig(sys.A);

% Determine controlability
co = ctrb(sys);
controllability = rank(co)

% LQR Design
Q = sys.C'*sys.C;
Q(1,1) = 100;
Q(2,2) = 500;
R = 1;
[K,~,e] = lqr(sys,Q,R)

% Find Nbar to eliminate steady state error
Cn = [1 0 0 0]; % Only affect input of rotor
sys_ss = ss(sys.A,sys.B,Cn,0);
Nbar = rscale(sys_ss,K)

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

sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
step(sys_cl)
figure
t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Precompensation and LQR Control')