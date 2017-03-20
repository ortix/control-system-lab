% close all;
%% Run file for the LQR simulation of the model
addpath('../../')
addpath('plant')

%% Load the linearization
x0 = [pi; 0; 0; 0; 0];
if ~exist('G_with_torque')      % Now around [pi; 0; 0; 0; 0]
    Pendulum_linid;
end

% state0 = [0 0 0 0];
% if ~exist('G_linearized')
%     G_linearized = linearization_by_hand(state0);
% end

G = G_with_torque;
params;
% h1 = 0.09;                          % Based on h*wc = 0.15, wc obtained from bode
% h2 = 0.0429;                        % Based on wc = 3.5
h3 = 0.01;

button = questdlg('Which model do you want to run?', 'Choose model','Continuous','Discrete','Continuous');
switch button
    case 'Continuous'
        fprintf('You are running the continuous model.\r')
        file = 'PP_model_cont_dd';
        open(file)
        %% Pole placement controller
        Overshoot = 0.01;   % 5 percent
        t_set = 0.5;          % 0.7 was too fast
        zeta = -log(Overshoot)/(sqrt(pi^2 + log(Overshoot)^2));
        omega = 5/(zeta*t_set);
        syms s 
        equation = s^2 + 2*zeta*omega*s + omega^2 == 0;
        poles = double(solve(equation,s));
        alpha = 5;
        lambdas = [poles(1) poles(2) real(poles(1))*alpha real(poles(1))*alpha-0.01 real(poles(1))*alpha+0.01];
        K = place(G.a, G.b,lambdas);
        eig(G.a - G.b*K)
        
        %% Build the observer
        % Observer gain should at least be ten times as fast as the slowest
        % pole of the pole placement controller
        Overshoot = 0.001;   % 5 percent
        t_set = 0.1;
        zeta = -log(Overshoot)/(sqrt(pi^2 + log(Overshoot)^2));
        omega = 5/(zeta*t_set);
        syms s 
        equation = s^2 + 2*zeta*omega*s + omega^2 == 0;
        poles = double(solve(equation,s));
        alpha = 7;
        lambdas = [poles(1) poles(2) real(poles(1))*alpha real(poles(1))*alpha-0.01 real(poles(1))*alpha+0.01];
        L = place(G_with_torque.a.', G_with_torque.c.',lambdas).';
        eig(G_with_torque.a - L*G_with_torque.c)
        
        %% Running the simulation
        x0 = [pi; 0*pi; 0; 0; 0];
        states = Simulink.BlockDiagram.getInitialState(file);
        theta1 = x0(1); theta2 = x0(2);
        states.signals(4).values = theta1;          % Initial state theta1 for visualizer
        states.signals(6).values = theta2;          % Initial state theta2 for visualizer
        
        t1 = 10;
        error = 1e-3;
        options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
        sim(file,t1,options);
        [t1,x,y,input] = sim(file,t1,options);
        theta1 = y(:,1); theta2 = y(:,2);
        return
     
        
    case 'Discrete'
        fprintf('You are running the discrete model.\r')
        
end

if ~exist('sysd1')
    discr_sys = c2d(G,h3);
end

e = eig(discr_sys);