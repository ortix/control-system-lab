params;

file = 'Pendulum_model'

open_system(file)
% states = [0 0 0 0 0]
states = Simulink.BlockDiagram.getInitialState(file)
theta1_0 = pi; theta2_0 = 0; theta1_dot_0 = 0; theta2_dot_0 = 0;
states.signals(1).values = [theta1_0 theta2_0];
states.signals(3).values = [theta1_dot_0 theta2_dot_0];
states.signals(2).values = 0;
% power_init(file,'x0',x0); % set the initial state of the simulink model
t1 = 10;
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
[t1,x,theta,torque] = sim(file,t1,options);
theta1 = theta(:,1); theta2 = theta(:,2);
% draw_pendulum

% figure()
% plot(t,torque)

x0 = [pi 0 0 0 0];
u0 = 0;

[x0,u0] = trim(file,x0',u0,[],1)
[A,B,C,D] = linmod(file,x0,u0);
G = ss(A,B,C,D)

% x0 = [0.5*pi 0.25*pi 0 0]; % theta1, theta2, thetadot1, thetadot2
% tspan = [0 10];
% [t2,state] = ode45(@pendulum_solver,tspan,x0);
% 
% theta1 = state(:,1); theta2 = state(:,2);
% draw_pendulum
% Pendulum_model