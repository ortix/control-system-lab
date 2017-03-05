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

%% Tune system%% Design Requirements
% Use |TuningGoal| requirements to specify the desired closed-loop behavior. 
% Specify a response time of 3 seconds for tracking a setpoint change in 
% cart position $x$.

% Tracking of x command
req1 = TuningGoal.Tracking('theta1_ref','theta1',3);

%%
% To adequately reject impulse disturbances $dF$ on the tip of
% the pendulum, use an LQR penalty of the form
%
% $$ \int_0^\infty (16 \theta^2(t) + x^2(t) + 0.01 F^2(t)) dt $$
%
% that emphasizes a small angular deviation $\theta$ and limits the control effort $F$.

% Rejection of impulse disturbance
Qxu = diag([16 1 0.1]);
req2 = TuningGoal.LQG('Disturbance',{'theta2','theta1','Torque'},1,Qxu);

%%
% For robustness, require at least 6 dB of gain margin and 40 degrees
% of phase margin at the plant input.

% Stability margins
req3 = TuningGoal.Margins('Torque',6,40);

%%
% Finally, constrain the damping and natural frequency of the closed-loop
% poles to prevent jerky or underdamped transients.

% Pole locations
MinDamping = 0.5;
MaxFrequency = 45;
req4 = TuningGoal.Poles(0,MinDamping,MaxFrequency);

%% Control System Tuning
% The closed-loop system is unstable for the initial values of the PD and
% state-space controllers (1 and $2/s$, respectively). You can use
% |systune| to jointly tune these two controllers. Use the |slTuner| 
% interface to specify the tunable blocks and register the plant 
% input |F| as an analysis point for measuring stability margins.

ST0 = slTuner('visualize',{'Theta1 Controller','Angle Controller'});
addPoint(ST0,'Torque');

%%
% Next, use |systune| to tune the PD and state-space controllers subject
% to the performance requirements specified above. Optimize the
% tracking and disturbance rejection performance (soft requirements)
% subject to the stability margins and pole location constraints
% (hard requirements).

rng(0)
Options = systuneOptions('RandomStart',5);
[ST, fSoft] = systune(ST0,[req1,req2],[req3,req4],Options);

%%
% The best design achieves a value close to 1 for the soft requirements
% while satisfying the hard requirements (|Hard|<1). This means that
% the tuned control system nearly achieves the target performance for
% tracking and disturbance rejection while satisfying the stability margins 
% and pole location constraints. 

%% Validation
% Use |viewSpec| to further analyze how the best design fares against each 
% requirement.
figure('Position',[100   100   575   660]);
viewSpec([req1,req3,req4],ST)

%%
% These plots confirm that the first two requirements are nearly satisfied
% while the last two are strictly enforced. Next, plot the responses to 
% a step change in position and to a force impulse on the cart.

T = getIOTransfer(ST,{'theta1_ref','Disturbance'},{'theta1','theta2'});
figure('Position',[100   100   650   420]);
subplot(121), step(T(:,1),10)
title('Tracking of set point change in position')
subplot(122), impulse(T(:,2),10)
title('Rejection of impulse disturbance')

%%
% The responses are smooth with the desired settling times. Inspect the 
% tuned values of the controllers.

C1 = getBlockValue(ST,'Position Controller')

%%

C2 = zpk(getBlockValue(ST,'Angle Controller'))

%%
% Note that the angle controller has an unstable pole that pairs up with
% the plant unstable pole to stabilize the inverted pendulum. To see this,
% get the open-loop transfer at the plant input and plot the root locus.

L = getLoopTransfer(ST,'Torque',-1);
figure;
rlocus(L)
set(gca,'XLim',[-25 20],'YLim',[-20 20])

%%
% To complete the validation, upload the tuned values to Simulink and
% simulate the nonlinear response of the cart/pendulum assembly.
% A video of the resulting simulation appears below.

writeBlockValue(ST)