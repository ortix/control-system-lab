function [sys,x0,str,ts] = sfusbout(t,x,u,flag,Ts,h)
%SFUSBIN USB output from Simulinka to a DCSC setup
%    Ts ... sampling period in [s]
%    h  ... handle to the USB serial port (obtained in hwinit.m)

%   (c) Robert Babuska, March 2005

switch flag
  case 0; [sys,x0,str,ts]=mdlInitializeSizes(Ts,h);
  case 3; sys = mdlOutputs(u,x,Ts,h);
  case 9; sys = mdlTerminate(h);
  otherwise; sys=[];
end

function [sys,x0,str,ts]=mdlInitializeSizes(Ts,h)
sizes = simsizes;
sizes.NumContStates  = 0;           % no need for states
sizes.NumDiscStates  = 0;           % no need for states
sizes.NumOutputs     = 0;           % no outputs
sizes.NumInputs      = 1;           % actuators
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = []; str = [];
ts = [-1 0];                        % Set sample time
if length(Ts)>0 ts(1)=Ts(1); end;
if length(Ts)>1 ts(2)=Ts(2); end;
tic;

function sys = mdlOutputs(u,x,Ts,h)
fugiboard('Write', h, 0, 1, u, 0);  % send actuator data to setup
while toc < Ts, end;                % synchronize with real time
tic;                                % reset Matlab's tic-toc timer
sys = [];

function sys = mdlTerminate(h)
fugiboard('Write', h, 0, 1, 0, 0);          % reset all actuators to 0
fugiboard('Close', h)
sys = [];
