% close all;
%% Run file for the LQR simulation of the model version 1, first observer gain and then state feedback gain
addpath('../')
addpath('plant')

if ~exist('G_with_torque')
    Pendulum_linid;
end

state0 = [0 0 0 0];
if ~exist('G_linearized')
    G_linearized = linearization_by_hand(state0);
end

G = G_linearized;

hwinit
calib

params;
h = 0.01;

if ~exist('sysd1')
    discr_sys = c2d(G,h);
end
discr_sys.c(2,:) = [1 1 0 0 0];

e = eig(discr_sys);

%% Building the observer gain
slowest_eig = max(e);
a = -0.05;
observer_poles = [0.2+a 0.21+a 0.22+a 0.23+a 0.24+a];
L = place(discr_sys.a',discr_sys.c',observer_poles)';

discr_closed_loop_L = ss(discr_sys.a-L*discr_sys.c,discr_sys.b,discr_sys.c,discr_sys.d);
eig(discr_closed_loop_L);

%% Creating the feedback gains by use of LQR
Q = diag([1e2 1e2 1 1 0]);
R = 1e1;
N = [0;0;0;0;0];

[K,S,e] = dlqr(discr_closed_loop_L.a,discr_closed_loop_L.b,Q,R,N);
discr_closed_loop_L_K = ss(discr_closed_loop_L.a-discr_closed_loop_L.b*K,discr_sys.b,discr_sys.c,discr_sys.d);
eig(discr_closed_loop_L_K)
K

% %% Running the simulink model
% file = 'LQR_control_model'
% 
% states_sys = [0 0 0 0 0];
% states_visualizer = [0 0 0 0 0 0];
% states = [states_sys states_visualizer];
% states = Simulink.BlockDiagram.getInitialState(file)
% x0 = [0 0.1 0 0 0]
% states.signals(5).values = x0;
% 
% t1 = 100;
% error = 1e-3;
% options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
% % sim(file,t1,options);
% [t1,x,y,input] = sim(file,t1,options);
% theta1 = y(:,1); theta2 = y(:,2);
% % draw_pendulum
% 
% figure(2)
% subplot(2,1,1)
% stairs(t1,theta1); 
% subplot(2,1,2)
% stairs(t1,theta2)
% 
% figure(3)
% stairs(t1,input)