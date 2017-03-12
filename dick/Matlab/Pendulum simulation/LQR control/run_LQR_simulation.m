% close all;
%% Run file for the LQR simulation of the model
addpath('../')
addpath('plant')

%% Discretize the system
if ~exist('G_with_torque')
    Pendulum_linid;
end

state0 = [0 0 0 0];
if ~exist('G_linearized')
    G_linearized = linearization_by_hand(state0);
end

G = G_with_torque;

file = 'LQR_control_model';

params;
h1 = 0.09;                          % Based on h*wc = 0.15, wc obtained from bode
h2 = 0.0429;                        % Based on wc = 3.5
h3 = 0.002;
 
if ~exist('sysd1')
    discr_sys = c2d(G,h3);
end

e = eig(discr_sys);

%% Creating the feedback gains by use of LQR
Q = diag([1 8000 0 0.5 1]);          % Input of system is [theta1 theta2 theta1dot theta2dot torque] Nick: [1 1 8000 0 0.5]
R = 0.1;                               % Nick: R = 1
N = [0;0;0;0;0];

% Compute the feedback gain K based on A, B, Q, R and N in discrete time
[K,S,e_cl] = dlqr(discr_sys.a,discr_sys.b,Q,R,N);
discr_sys_cl_K.a = discr_sys.a - discr_sys.B*K;     % Add the feedback gain to make a discrete closed loop system instead of continuous! 
return
discr_cl_LQR = c2d(ss(Ac,discr_sys.b,discr_sys.c,discr_sys.d),h3);
e_cl
K
figure(); impulse(discr_cl_LQR)

% Compute the feedback gain K based on A, B, Q, R and N in continuous time
[K,S,e_cl] = lqr(G_with_torque.a,G_with_torque.b,Q,R,N);
Ac = G_with_torque.a - G_with_torque.b*K;
cont_cl_LQR = ss(Ac,G_with_torque.b,G_with_torque.c,G_with_torque.d);
discr_sys_probeersel = c2d(cont_cl_LQR,h3); 
K
figure(); impulse(cont_cl_LQR)
eig(discr_sys_probeersel)
figure(); impulse(discr_sys_probeersel)
return
%% Building the observer gain
slowest_eig = max(e);
a = -0;
observer_poles = [0.2+a 0.21+a 0.22+a 0.23+a 0.24+a];
L = place(discr_sys.a',discr_sys.c',observer_poles)';

discr_closed_loop_L = ss(discr_sys.a-L*discr_sys.c,discr_sys.b,discr_sys.c,discr_sys.d);
eig(discr_closed_loop_L);

%% Running the simulation
states_sys = [0 0 0 0 0];
states_visualizer = [0 0 0 0 0 0];
states_observer = [0 0 0 0 0];
states = [states_sys states_visualizer states_observer];
states = Simulink.BlockDiagram.getInitialState(file);
theta1 = 0; theta2 = 0; theta = [theta1 theta2];
states.signals(1).values = theta;           % Initial states for theta of nonlinear model
states.signals(2).values = theta1;          % Initial state theta1 for visualizer
states.signals(4).values = theta2;          % Initial state theta2 for visualizer
states.signals(8).values = [theta 0 0 0];   % observer states

t1 = 3;
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
sim(file,t1,options);
[t1,x,y,input] = sim(file,t1,options);
theta1 = y(:,1); theta2 = y(:,2);
 
figure(2)
subplot(2,1,1)
stairs(t1,theta1); 
subplot(2,1,2)
stairs(t1,theta2)

figure(3)
stairs(t1,input)