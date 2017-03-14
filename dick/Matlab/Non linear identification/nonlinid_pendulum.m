clear all; clc
%% Nonlinear identification of parameters of the pendulum model
% First an approximation of the parameters of the first link is made.
% Secondly the found parameters are used to find the parameters of the
% second link

%% Generate a crested multisine input to excite the system with
addpath('Basic plant')
% calib
% hwinit;
% See multsine_generation for parameters
Multisine_generation;
h = dt;   % sample time in seconds
U = [t xpc2_long]; 
crinfo
figure(3)
plot(U(:,1),U(:,2))
% % input for measurement noise
% U = [t zeros(length(t),1)];

% %% Generate a GBN input signal to excite the system with
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % design input excitation signal
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T = 10;    % length of experiment
% h = 0.01;   % sampling interval
% ts = 0.1;    % estimated settling time of the process
% A = 0.1;      % amplitude of GBN
% U = [h*(0:T/h)' gbn(T,ts,A,h,1)];
return
%% Loading the input data
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\Crested Multisine')
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\GBN')

input = 'multisine';

switch input
    case 'GBN'
GBN_input = load('GBN_input1');
GBN_output = load('GBN_output1');
time = GBN_input.ans.Time;
h = diff(time(1:2));
U = GBN_input.ans.Data;
y = GBN_output.ans.Data;
    case 'multisine'
multisine_input  = load('cr_multisine_input_2');
multisine_output = load('cr_multisine_output_2');
time = multisine_input.ans.Time;
h = diff(time(1:2));
U = multisine_input.ans.Data;
y = multisine_output.ans.Data;
end

% U should be timeseries
U = [time U];
figure(2); p1 = plot(time,y(:,1)); p2 = plot(time,y(:,2)); hold on;
xlabel('Time [sec]')
ylabel('angle [rad]')
title('Output of the real system and the simulation model for the same input')
%% Non linear parameter estimation
% % When estimating parameters of the first link: 
% % initial guess: [l1 m1 c1 I1 b1 km tau_e]
parameters_initial_guess = [4.8 0.0002 50 0.3].';   % initial guess
k_m = 50;
tau_e = 0.3;
x0 =[pi 0 0 0 0];

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % design input excitation signal
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T = 10;    % length of experiment
% h = 0.01;   % sampling interval
% ts = 0.1;    % estimated settling time of the process
% A = 0.1;      % amplitude of GBN
% U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
assignin('base','k_m',k_m)
sim('Pendulum_model_nlsysid');     % ouput data available in y

figure(2); p3 = plot(U(:,1),y(:,1)); p4 = plot(U(:,1),y(:,2));
legend([p1,p2,p3,p4],'theta1 real system','theta2 real system','theta1 simulation model','theta2 simulation model')
return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lb = [0 0 0 0]; up = [];
OPT = optimoptions(@lsqnonlin,'MaxIterations',50);
[xhat,fval]= lsqnonlin('costfun_pendulum',parameters_initial_guess,lb,up,OPT,U,y);
[parameters_initial_guess xhat], fval