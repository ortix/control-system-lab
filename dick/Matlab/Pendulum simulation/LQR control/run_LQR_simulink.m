% close all;
%% Run file for the LQR simulation of the model version 2, first state feedback gain and then observer gain
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
h = 0.002;

if ~exist('sysd1')
    discr_sys = c2d(G,h);
end
discr_sys.c(2,:) = [0 1 0 0 0];

e = eig(discr_sys);

%% Creating the feedback gains by use of LQR
Q = diag([1e1 1e5 1 1 0]);
R = 1e10;
N = [0;0;0;0;0];

[K,S,e] = dlqr(discr_sys.a,discr_sys.b,Q,R,N);
discr_closed_loop_K = ss(discr_sys.a-discr_sys.b*K,discr_sys.b,discr_sys.c,discr_sys.d);
eig(discr_closed_loop_L_K)
K
%% Building the observer gain
slowest_eig = max(e);
a = -0;
observer_pole = slowest_eig/10;
observer_poles = [observer_pole observer_pole+0.01 observer_pole-0.01 observer_pole-0.02 observer_pole+0.02];
L = place(discr_closed_loop_K.a',discr_closed_loop_K.c',observer_poles)';

discr_closed_loop_K_L = ss(discr_closed_loop_K.a-L*discr_closed_loop_K.c,discr_sys.b,discr_sys.c,discr_sys.d);
eig(discr_closed_loop_K_L)


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