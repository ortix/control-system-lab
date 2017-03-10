% close all;
%% Run file for the LQR simulation of the model
addpath('../')

if ~exist('G_with_torque')
    Pendulum_linid;
end

params;
state0 = [0; 0; 0; 0];             % initial state
h1 = 0.09;                          % Based on h*wc = 0.15, wc obtained from bode
h2 = 0.0429;                        % Based on wc = 3.5
h3 = 0.01;

if ~exist('G_linearized')
    G_linearized = linearization_by_hand(state0)
end

G = G_with_torque;

if ~exist('sysd1')
    sysd1 = c2d(G,h1);
    sysd2 = c2d(G,h2);
    sysd3 = c2d(G,h3);
    
    figure(1)
    impulse(G_linearized,'-y',sysd1,'--r',sysd2,'--g',sysd3,'--k',2);
    legend('Continuous','h=0.09','h=0.17')
end

discr_sys = sysd3;

%% Creating the feedback gains by use of LQR
Q = diag([1 1 1 1 0]);
R = 1;
N = [0;0;0;0;0];

[K,S,e] = dlqr(discr_sys.a,discr_sys.b,Q,R,N);

%% Running the simulink model
file = 'LQR_control_model'

states_sys = [0 0 0 0 0];
states_visualizer = [0 0 0 0 0 0];
states = [states_sys states_visualizer];
states = Simulink.BlockDiagram.getInitialState(file)
x0 = [0 0.1 0 0 0]
states.signals(5).values = x0;

t1 = 100;
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
% sim(file,t1,options);
[t1,x,y,input] = sim(file,t1,options);
theta1 = y(:,1); theta2 = y(:,2);
% draw_pendulum

figure(2)
subplot(2,1,1)
stairs(t1,theta1); 
subplot(2,1,2)
stairs(t1,theta2)

figure(3)
stairs(t1,input)