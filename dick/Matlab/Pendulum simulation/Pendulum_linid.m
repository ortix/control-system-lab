% Pendulum linear model identification 
% Estimate a linear input-output model
clc;
params;
file = 'Pendulum_model';

x0 = [pi;0;0;0;0]; % state at the operating point
states = x0;
u0 = 0;
% states = Simulink.BlockDiagram.getInitialState(file)
% theta1_0 = pi; theta2_0 = 0; theta1_dot_0 = 0; theta2_dot_0 = 0;
% states.signals(1).values = [theta1_0 theta2_0];
% states.signals(3).values = [theta1_dot_0 theta2_dot_0];
% states.signals(2).values = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linearize nonlinear model around x0, u0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[x0,u0,y0] = trim(file,x0,u0,[],1);
[A,B,C,D] = linmod(file,x0,u0);
G = ss(A,B,C,D);

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
sim('Pendulum_model_linid');  % ouput data available in y
U = U(:,2);
y = y.data(:,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linear identification
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1); clf;
data = iddata(y,U,h);
nb = [5]; nf = [5]; nk = [1];
M = oe(data,[nb nf nk]); subplot(3,1,1); compare(data,M);
na = [5]; nc = [1];
L = armax(data,[na nb nc nk]); subplot(3,1,2); compare(data,L);
% F = arx(data,[na nb nk]); subplot(3,1,3); compare(data,F);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compare with linearized model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2); clf;
Ghat = ss(M); Ghat = Ghat(1,1);
Lhat = ss(L); Lhat = Lhat(1,1);
Fhat = ss(F); Fhat = Fhat(1,1);
figure(3)
step(G,Ghat);
figure(4)
step(G,Lhat);
tf(d2c(Ghat))
tf(d2c(Lhat))
tf(G)