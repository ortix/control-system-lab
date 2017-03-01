%%% Rotational Pendulum
%% Open up the system
model = 'visualize';
load_system(model);

%% Define extra parameters for model
par.initial_state = [0 0]; % Pendulum is vertical (up up)
par.torque = 0; % Input torque
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