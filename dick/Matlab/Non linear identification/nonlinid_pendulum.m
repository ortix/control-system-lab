clear all; clc
%% Nonlinear identification of parameters of the pendulum model
% First an approximation of the parameters of the first link is made.
% Secondly the found parameters are used to find the parameters of the
% second link

%% Generate a crested multisine input to excite the system with
addpath('Basic plant')
calib
hwinit;
T = 10;     % duration of expirement
h = 0.01;   % sample time in seconds
t = h*(0:T/h)';
Multisine_generation;
U = [t xpc2_long]; 
crinfo

% input for measurement noise
U = [t zeros(length(t),1)];

return
% addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\GBN')
% GBN_input = load('GBN_input1');
% GBN_output = load('GBN_output1');
% 
% time = GBN_input.ans.Time;
% h = diff(time(1:2));
% U = GBN_input.ans.Data;
% y = GBN_output.ans.Data;
% 
% fs = 100;                        % sampling frequency
% f = (1:length(time))./time(end); % Create a frequency vector
% 
% U_fft = fft(U);
% figure(1);
% subplot(2,1,1);
% plot(time,U)
% subplot(2,1,2);
% loglog(f,abs(U_fft))


%%
b = .5;     % "true" parameter value
bi = 0.2;   % initial guess
x0 = [0;0]; % initial state

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% design input excitation signal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = 100;    % length of experiment
h = 0.05;   % sampling interval
ts = 10;    % estimated settling time of the process
A = 1;      % amplitude of GBN
U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sim('nlsysid');     % ouput data available in y

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
OPT = optimset('MaxIter',25);
[bhat,fval]= lsqnonlin('costfun',bi,0,[],OPT,U,y);
[b bhat], fval