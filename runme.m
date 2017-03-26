%%% Rotational Pendulum
if(~exist('runflag','var'))
clear;clc;close all
end
addpath('plant')
addpath('libs')
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

%% Check controllability
disp('Checking controllability');

% Find the poles
poles = eig(sys.A);

% Determine controlability
co = ctrb(sys);
controllability = rank(co);
if(controllability == length(sys.b))
    disp('System is controllable')
else
    error('System is not controllable')
end

controller = questdlg('Which controller do you want to use?','Choose controller','LQR controller','Pole placement controller','LQR controller');

switch controller
    case 'LQR controller'
        %% LQR Pole Placement
        % LQR Designsys
        Q = sys.C'*sys.C;
        
        %%%%% Optimize this %%%%%%%%%%
        Q = diag([1 10000 100000 100 1]);
        R = 60000;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        disp('Calculating LQR poles');
        [K,~,e] = dlqr(sys.A,sys.B,Q,R);
        Kff = 1.05;
        
    case 'Pole placement controller'
        %% Pole placement controller
        disp('Calculating pole placement poles');
        % poles = [0 0.7673+0.0563i 0.7673-0.0563i 0.9454 0.9195]; % Correct poles from LQR controller
        overshoot = 0.05;
        t_set = 0.7;
        zeta = -log(overshoot)/(sqrt(pi^2 + log(overshoot)^2));
        omega = 4.6/(zeta*t_set);
        p1 = -2*exp(-2*zeta*omega*Ts)*cos(omega*Ts*sqrt(1-zeta^2));
        p2 = exp(-2*zeta*omega*Ts);
        
        syms s
        equation = s^2 + p1*s + p2 == 0;
        poles = double(solve(equation,s));
        alpha = 0.25;
        
        % [torque theta1 theta2 theta1dot theta2dot]
        lambdas = [0; poles(1); poles(2);
            0.001+real(poles(1))^alpha;
            -0.001+real(poles(1))^alpha].';
        lambdas
        K = place(sys.A,sys.B,lambdas);
        Kff = 0.75;
end
%% Close loop
% Create new state space representation with full state feedback by
% using K found with LQR or Pole Placement
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


%% Observer
observability = rank(obsv(sys));

% Check stability of system with state feedback
poles = eig(sys_cl)
lambda = max(poles)/10;

% Place the poles
P = [0 lambda+0.001 lambda+0.002 lambda+0.003 lambda+0.004]
L = place(sys.A.',sys.C.',P).'

% Controller
Ace = [(sys.A-sys.B*K) (sys.B*K); zeros(size(sys.A)) (sys.A-L*sys.C)];
Bce = [sys.B; zeros(size(sys.B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states = {'theta1' 'theta2' 'theta1_dot' 'theta2_dot' ...
    'T_dot' 'e1' 'e2' 'e3' 'e4' 'e5'};
inputs = {'torque'};
outputs = {'theta1'; 'theta2'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs, ...
    'outputname',outputs);
%% Save the state matrices
state.A = sys.a;
state.B = sys.b;
state.C = sys.c;
state.D = sys.d;
state.K = K;
state.h = Ts;
state.Ts = Ts;
state.L = L;
state.initial_state = params.initial_state;
if strcmp(controller,'LQR controller')
    state.Q = Q;
    state.R = R;
    runflag = 1;
end
toModelWorkspace('visualize',state);

toModelWorkspace('rpend',state);

%% Simulate
% LQR controller has to be calculated before simulation can run
if(~exist('runflag','var'))
    return
end
disp('Running simulation...');
evalc('sim(''visualize'')');
disp('Simulation complete!');
return
%% Plot graphs
disp('Creating pretty graphs')
% Theta1
figure('units','normalized','position',[0.2 0.2 0.4 0.4]);
subplot(2,2,1);hold on;grid on;
plot(yhat.time,yhat.Data(:,1),':','linewidth',2)
plot(yhat.time,yhat.Data(:,3),'-','linewidth',2)
title('Theta 1 angle')
ylabel('amplitude [rad]');xlabel('time [s]')
legend('unfiltered','filtered','Location','south')

subplot(2,2,2);hold on;grid on;
plot(yhat.time,yhat.Data(:,1),':','linewidth',2)
plot(yhat.time,yhat.Data(:,3),'-','linewidth',2)
title('Theta 1 angle')
ylabel('amplitude [rad]');xlabel('time [s]')
ylim([-0.01 0.01]);xlim([0.5 1])
legend('unfiltered','filtered','Location','south')

%Theta2
subplot(2,2,3);hold on; grid on;
plot(yhat.time,yhat.Data(:,2),':','linewidth',2)
plot(yhat.time,yhat.Data(:,4),'-','linewidth',2)
title('Theta 2 angle')
ylabel('amplitude [rad]');xlabel('time [s]')
legend('unfiltered','filtered','Location','south')

subplot(2,2,4);hold on; grid on;
plot(yhat.time,yhat.Data(:,2),':','linewidth',2)
plot(yhat.time,yhat.Data(:,4),'-','linewidth',2)
title('Theta 2 angle')
ylabel('amplitude [rad]');xlabel('time [s]')
ylim([-0.01 0.01]);xlim([0.5 1])
legend('unfiltered','filtered','Location','south')
return
% Disturbance
subplot(3,1,3);hold on; grid on;
plot(disturbance.time,disturbance.Data(:,2),'LineWidth',1)
title('Disturbance')
ylabel('amplitude [rad]');xlabel('time [s]')
saveas(gcf,'../presentation/img/disturbance_effect_kalman','png');

% Torque
figure('units','normalized','position',[0.3 0.3 .4 0.3]);
plot(torque,'linewidth',0.7);
title('Control Effort')
xlabel('time [s]')
ylabel('torque [N]')
grid on;
ylim([-1 1])
saveas(gcf,'../presentation/img/torque','png');

disp('I can haz good grade?')


