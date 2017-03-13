% estimate (fine-tune) a parameter in a nonlinear Simulink model

clear all; clc;
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