% close all;
%% Run file for the LQR simulation of the model
addpath('../')
addpath('plant')

%% Discretize the system
if ~exist('G_with_torque')      % Now around [pi; 0; 0; 0; 0]
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
h3 = 0.005;
 
if ~exist('sysd1')
    discr_sys = c2d(G,h3);
end

e = eig(discr_sys);

%% Creating the feedback gains by use of LQR
Q = diag([15 8000 1 0.5 1]);          % Input of system is [theta1 theta2 theta1dot theta2dot torque] Nick: [1 1 8000 0 0.5]
R = 1;                                % Nick: R = 1
N = [0;0;0;0;0];

% Compute the feedback gain K based on A, B, Q, R and N in discrete time
[K,S,e_cl] = dlqr(discr_sys.a,discr_sys.b,Q,R,N);
discr_cl_K = discr_sys; discr_cl_K.a = discr_sys.a - discr_sys.B*K;   % Add the feedback gain to make a discrete closed loop system instead of continuous! 
e_cl
K
figure(1); impulse(discr_cl_K)

% %% Creating the feedback gain using pole placement using state feedback
% zeta = 1;                       % Overshoot of around 5%
% w = 1/1*4.6/(zeta*0.1)*1;       % settling time of 0.1 seconds, % 1 is original
% p1 = -2*exp(-2*zeta*w*h3)*cos(w*h3*sqrt(1-zeta^2));
% p2 = exp(-2*zeta*w*h3);
% N = 2*pi/(w*h3*sqrt(1-zeta^2));
% 
% syms s
% b = s^2 + p1*s + p2;
% lambda = solve(b,s);
% lambda = single(lambda);
% pole1 = [lambda(1) lambda(2) 0.8 0.7];              % Arbitrary pole locations
% pole2 = [lambda(1) lambda(2) 0.6 0.5];
% pole3 = [lambda(1) lambda(2) 0.4 0.3];
% pole4 = [lambda(1) lambda(2) 0.2 0.1];
% pole_saturated =[lambda(1) lambda(2) 0.46 0.44 0.45];
% pole_delay = [lambda(1) lambda(2) 0.75 0.76];
% 
% pole_saturated = [0.6 0.65 0.62 0.7 0.8];
% K = place(discr_sys.a,discr_sys.b,pole_saturated);
% e_cl = discr_sys.a - discr_sys.b*K;
% eig(closedloop)

%% Building the observer gain
slowest_eig = max(e_cl);
a = 0.1;
observer_poles = [0.05+a 0.04+a 0.02+a 0.03+a 0.04+a];
% observer_poles = [0.1 0.15 0.2 0.65 0.6];
L = place(discr_sys.a',discr_sys.c',observer_poles)';

discr_cl_L = discr_sys; discr_cl_L.a = discr_sys.a - L*discr_sys.c;
eig(discr_cl_L)
figure(2); impulse(discr_cl_L)
eig_total_system = eig(discr_cl_K.a - L*discr_sys.c)

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

% K = -K; % Otherwise simulation is rotating to exact the wrong way

t1 = 3;
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
sim(file,t1,options);
[t1,x,y,input] = sim(file,t1,options);
theta1 = y(:,1); theta2 = y(:,2);
 
figure(3)
subplot(2,1,1)
stairs(t1,theta1); 
title('theta1')
subplot(2,1,2)
stairs(t1,theta2)
title('theta2')

figure(4)
stairs(t1,input)