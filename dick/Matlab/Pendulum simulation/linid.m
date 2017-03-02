% estimate a linear input-output model

clear all; clc;
b = .5;     % "true" parameter value
x0 = [2;0]; % state at the operating point

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linearize nonlinear model around x0, u0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[x0,u0,y0] = trim('nlsys',x0,[],[],1);
[A,B,C,D] = linmod('nlsys',x0,u0);
G = ss(A,B,C,D);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% design input excitation signal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = 50;     % length of experiment
h = 0.1;    % sampling interval
ts = 10;    % estimated settling time of the process
A = 0.1;    % amplitude of GBN
U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sim('lsysid');  % ouput data available in y

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linear identification
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1); clf;
data = iddata(y,U(:,2),h);
M = oe(data,[2 2 1]); subplot(3,1,1); compare(data,M);
L = armax(data,[2 2 1 0]); subplot(3,1,2); compare(data,L);
F = arx(data,[2 2 1]); subplot(3,1,3); compare(data,F);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compare with linearized model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2); clf;
Ghat = ss(M); Ghat = Ghat(1,1);
step(G,Ghat);
tf(d2c(Ghat))
tf(G)
