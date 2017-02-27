function [sys,x0,str,ts] = sfusbin(t,x,u,flag,Ts,h, channels)
%SFUSBIN USB input from a DCSC setup into Simulink.
%    Ts ... sampling period in [s]
%    h  ... handle to the USB serial port (obtained in hwinit.m)

%   (c) Robert Babuska, January 2008

switch flag
  case 0; [sys,x0,str,ts]=mdlInitializeSizes(Ts,h);
  case 3; sys = mdlOutputs(u,x,h, channels);
  case 9; sys = mdlTerminate(h);
  otherwise; sys=[];
end

function [sys,x0,str,ts]=mdlInitializeSizes(Ts,h)
sizes = simsizes;
sizes.NumContStates  = 0;                   % no need for states
sizes.NumDiscStates  = 0;                   % no need for states
sizes.NumOutputs     = 7;                   % 2 sensors + 4 switches + 1 encoder
sizes.NumInputs      = 0;                   % no inputs
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = []; str = [];
ts = [-1 0];                                % Set sample time
if length(Ts)>0 ts(1)=Ts(1); end;
if length(Ts)>1 ts(2)=Ts(2); end;

function sys = mdlOutputs(u,x,h, channels)
d = fugiboard('Read', h);                           % read the data using MEX file
s = rem(floor(d(8)*[0.5 0.25 0.125 0.0625]),2)';    % convert to binary
sys = [d(channels); s; d(4)];                       % compose the vector to return

function sys = mdlTerminate(h)
fugiboard('Write', h, 0, 1, 0, 0);          % reset all actuators to 0
fugiboard('Close', h)
sys = [];
