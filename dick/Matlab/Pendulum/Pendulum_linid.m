% Pendulum linear model identification 
% Estimate a linear input-output model
clc;
% params;
file = 'Pendulum_model_linearization';
addpath('../')

% x0 = [pi;0;0;0]; % state at the operating point
% states = x0;
% u0 = 0;
% 
% k_m = 50;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % linearize nonlinear model around x0, u0
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Specify the model name
% select_input = 0;
% model = 'Pendulum_model_linearization';
% 
% %% Specify the operating point
% % Use the model initial condition
% op = operpoint(model);
% 
% %% Linearize the model
% sys = linearize(model,op);
% 
% %% Plot the resulting linearization
% % step(sys)
% G_no_torque = ss(sys);
% 
%% Linearize the system with the torque as a state
file = 'Pendulum_model_linearization_torque';
% x0 = [0;0;0;0;0]; % state at the operating point
states = x0;
u0 = 0;

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
clearvars -except G_with_torque
return