% close all;
%% Run file for the LQR simulation of the model
addpath('../')
addpath('plant')

if ~exist('G_with_torque')
    Pendulum_linid;
end

file = 'LQR_control_model';
hwinit
calib

params;
state0 = [0; 0; 0; 0];             % initial state
h1 = 0.09;                          % Based on h*wc = 0.15, wc obtained from bode
h2 = 0.0429;                        % Based on wc = 3.5
h3 = 0.001;
 
%% Building the observer gain
slowest_eig = max(e);
a = -0;
observer_poles = [0.2+a 0.21+a 0.22+a 0.23+a 0.24+a];
L = place(discr_sys.a',discr_sys.c',observer_poles)';

discr_closed_loop_L = ss(discr_sys.a-L*discr_sys.c,discr_sys.b,discr_sys.c,discr_sys.d);
eig(discr_closed_loop_L);

%% Creating the feedback gains by use of LQR
Q = diag([1 1e9 1 1 0]);
R = 1000000;
N = [0;0;0;0;0];

[K,S,e] = dlqr(discr_closed_loop_L.a,discr_closed_loop_L.b,Q,R,N);
discr_closed_loop_L_K = ss(discr_closed_loop_L.a-discr_closed_loop_L.b*K,discr_sys.b,discr_sys.c,discr_sys.d);
eig(discr_closed_loop_L_K)
K

states_sys = [0 0 0 0 0];
states_visualizer = [0 0 0 0 0 0];
states_observer = [0 0 0 0 0];
states = [states_sys states_visualizer states_observer];
states = Simulink.BlockDiagram.getInitialState(file)
x0 = [0 0.1 0 0 0]
states.signals(5).values = x0;
states.signals(6).values = x0;

t1 = 10;
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
% sim(file,t1,options);
[t1,x,y,input] = sim(file,t1,options);
% theta1 = y(:,1); theta2 = y(:,2);
% % draw_pendulum
% 
figure(2)
subplot(2,1,1)
stairs(t1,theta1); 
subplot(2,1,2)
stairs(t1,theta2)

figure(3)
stairs(t1,input)