% Pendulum linear model identification 
% Estimate a linear input-output model
clc;
params;
file = 'Pendulum_model_linearization';
addpath('../')

x0 = [pi;0;0;0]; % state at the operating point
states = x0;
u0 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linearize nonlinear model around x0, u0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Specify the model name
select_input = 0;
model = 'Pendulum_model_linearization';

%% Specify the operating point
% Use the model initial condition
op = operpoint(model);

%% Linearize the model
sys = linearize(model,op);

%% Plot the resulting linearization
% step(sys)
G_no_torque = ss(sys);

%% Linearize the system with the torque as a state
file = 'Pendulum_model_linearization_torque';
x0 = [pi;0;0;0;0]; % state at the operating point
states = x0;
u0 = 0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linearize nonlinear model around x0, u0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Specify the model name
select_input = 0;
model = 'Pendulum_model_linearization_torque';

%% Specify the operating point
% Use the model initial condition
op = operpoint(model);

%% Linearize the model
sys = linearize(model,op);

G_with_torque = ss(sys)

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % design input excitation signal
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = 10;     % length of experiment
h = 0.01;    % sampling interval
ts = 0.1;    % estimated settling time of the process
A = 0.1;    % amplitude of GBN
U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
select_input = 1;
sim('Pendulum_model_linid');  % ouput data available in y
U = U(:,2);
y = y.data(:,2);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % linear identification
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1); clf;
% data = iddata(y,U,h);
% nb = [4]; nf = [5]; nk = [1];
% M = oe(data,[nb nf nk]); subplot(3,1,1); compare(data,M);
% na = [5]; nc = [1];
% L = armax(data,[na nb nc nk]); subplot(3,1,2); compare(data,L);
% % F = arx(data,[na nb nk]); subplot(3,1,3); compare(data,F);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % compare with linearized model
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(2); clf;
% Ghat = ss(M); Ghat = Ghat(1,1);
% Lhat = ss(L); Lhat = Lhat(1,1);
% figure(3)
% step(G_with_torque,Ghat);
% figure(4)
% step(G_with_torque,Lhat);
% H11= tf(d2c(Ghat))
% H12= tf(d2c(Lhat))
% tf(G_with_torque)
% % 
% % % Obtaining the total transfer function 
% H_sys = [H11; H12];