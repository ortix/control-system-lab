clear all; clc
%% Nonlinear identification of parameters of the pendulum model
% First an approximation of the parameters of the first link is made.
% Secondly the found parameters are used to find the parameters of the
% second link

%% Generate a crested multisine input to excite the system with
addpath('Basic plant')
% calib
% hwinit;

% % See multsine_generation for parameters
% Multisine_generation;
% h = dt;   % sample time in seconds
% U = [t xpc2_long]; 
% crinfo
% figure(3)
% plot(U(:,1),U(:,2))
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

%% Loading the input data
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\Crested Multisine')
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\GBN')
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\Step')
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\Swing')

input = 'swing';

switch input
    case 'GBN'
        GBN_input = load('GBN_input3_Ts_01_metzachtterugduwen_A_015');
        GBN_output = load('GBN_output3_Ts_01_metzachtterugduwen_A_015');
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
    case 'step'
        step_input  = load('step_input1');
        step_output = load('step_output1');
        time = step_input.ans.Time;
        h = diff(time(1:2));
        U = step_input.ans.Data;
        y = step_output.ans.Data;
    case 'swing'
        swing_input  = load('input_swing');
        swing_output = load('output_swing2');
        time = swing_input.ans.Time;
        h = diff(time(1:2));
        U = swing_input.ans.Data;
        y = swing_output.ans.Data;
        % Shift the output data to correspond to the simulation data
        n = max(find(y(:,2)<0));
        y = y(n:end,:);
        index = length(y);
       
        y(index:index+n-1,1)  = mean(y(:,1));
        y(index:index+n-1,2)  = mean(y(:,2));
end

% % pre-process data, take detrend
% y = detrend(y);
% y(:,1) = pi + y(:,1);

% U should be timeseries
U = [time U];
figure(2); p1 = plot(time,y(:,1)); p2 = plot(time,y(:,2)); hold on;
xlabel('Time [sec]')
ylabel('angle [rad]')
title('Output of the real system and the simulation model for the same input')
%% Non linear parameter estimation
% % When estimating parameters of the first link: 
% % initial guess: [b1 b2 k_m tau_e]
% parameters_initial_guess = [4.8 0.0002 50 0.3].';   % initial guess

% Calculate damping term b2 from data
[x_max1, ind1] = max(y(1:50,2));
[x_max2, ind2] = max(y(50:end,2));
t_cycle = time(ind2+49)-time(ind1);
w_n = (2*pi)/t_cycle;
T_s = 25.7;
zeta = 4.6/(T_s*w_n);
b2 = 2*zeta*w_n;


% When estimating parameters of the first link:
% initial guess: [l2 m2 c2 I2 b2]
parameters_initial_guess = [0.1 0.05 0.06 0.000012 0.00002].';   % initial guess

k_m = 50;
tau_e = 0.3;
% Fill in initial guess. Link 1: x0 = [pi 0 0 0 0]
                       % Link 2: x0 = [0.5*pi 0 0 0 0]
x0 =[0.5*pi+0.001 -0.001 0 0 0];

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % design input excitation signal
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T = 10;           % length of experiment
% h = 0.01;         % sampling interval
% ts = 0.1;         % estimated settling time of the process
% A = 0.1;          % amplitude of GBN
% U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% assignin('base','k_m',k_m)
% sim('Pendulum_model_nlsysid',30);     % ouput data available in y
% 
% figure(2); p3 = plot(U(:,1),y(:,1)); p4 = plot(U(:,1),y(:,2));
% legend([p1,p2,p3,p4],'theta1 real system','theta2 real system','theta1 simulation model','theta2 simulation model')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lb = [0 0 -0.1 0 0]; up = [];
OPT = optimoptions(@lsqnonlin,'MaxIterations',15,'StepTolerance',1e-6);
[xhat,fval]= lsqnonlin('costfun_pendulum',parameters_initial_guess,lb,up,OPT,U,y);
[parameters_initial_guess xhat], fval