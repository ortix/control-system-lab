%%% Rotational Pendulum
%% Open up the system
model = 'visualize';
load_system(model);

%% Define extra parameters for model
par.initial_state = [0.5*pi 0.2*pi]; % Pendulum is vertical (up up)
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

sim(model)