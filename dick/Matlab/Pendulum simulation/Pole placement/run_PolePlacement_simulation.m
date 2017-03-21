% close all;
%% Run file for the LQR simulation of the model
addpath('../')
addpath('plant')

%% Load the linearization
x0 = [pi; 0;0;0;0];
if ~exist('G_with_torque')      % Now around [pi; 0; 0; 0; 0]
    Pendulum_linid;
end

state0 = [pi 0 0 0];
if ~exist('G_linearized')
    G_linearized = linearization_by_hand(state0);
end

G = G_with_torque;
params;
% h1 = 0.09;                          % Based on h*wc = 0.15, wc obtained from bode
% h2 = 0.0429;                        % Based on wc = 3.5
h3 = 0.01;


button = questdlg('Which model do you want to run?', 'Choose model','Continuous','Discrete','Continuous');
switch button
    case 'Continuous'
        fprintf('You are running the continuous model.\r')
        file = 'PolePlacement_control_model_continuous';
        
        %% Pole placement controller
        Overshoot = 0.05;   % 5 percent
        t_set = 1;          % 0.7 was too fast
        zeta = -log(Overshoot)/(sqrt(pi^2 + log(Overshoot)^2));
        omega = 5/(zeta*t_set);
        syms s 
        equation = s^2 + 2*zeta*omega*s + omega^2 == 0;
        poles = double(solve(equation,s));
        alpha = 2;
        lambdas = [real(poles(1))*alpha real(poles(1))*alpha-0.01 poles(1) poles(2) real(poles(1))*alpha+0.01];
        K = place(G.a, G.b,lambdas);
        eig(G.a - G.b*K)
        
        %% Build the observer
        % Observer gain should at least be ten times as fast as the slowest
        % pole of the pole placement controller
        Overshoot = 0.01;   % 5 percent
        t_set = 0.1;
        zeta = -log(Overshoot)/(sqrt(pi^2 + log(Overshoot)^2));
        omega = 5/(zeta*t_set);
        syms s 
        equation = s^2 + 2*zeta*omega*s + omega^2 == 0;
        poles = double(solve(equation,s));
        alpha = 7;
        lambdas = [real(poles(1))*alpha real(poles(1))*alpha-0.01 poles(1) poles(2) real(poles(1))*alpha+0.01];
        L = place(G_with_torque.a.', G_with_torque.c.',lambdas).';
        eig(G_with_torque.a - L*G_with_torque.c)
        
        %% Running the simulation
        x0 = [0; 0; 0; 0; 0];
        states = Simulink.BlockDiagram.getInitialState(file);
        theta1 = 0.1; theta2 = 0;
        states.signals(3).values = theta1;          % Initial state theta1 for visualizer
        states.signals(5).values = theta2;          % Initial state theta2 for visualizer
        
        t1 = 10;
        error = 1e-3;
        options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
        sim(file,t1,options);
        [t1,x,y,input] = sim(file,t1,options);
        theta1 = y(:,1); theta2 = y(:,2);
        return
     
        
    case 'Discrete'
        fprintf('You are running the discrete model.\r')
        file = 'PolePlacement_control_model_discrete';
        %% Pole placement controller
        discr_sys = c2d(G,h3);
        zeta = 0.7;
        t_set = 1;
        w = 4.6/(zeta*t_set);       % settling time of 8 seconds  % 5 is delay, % 2,5 is original
        p1 = -2*exp(-2*zeta*w*h3)*cos(w*h3*sqrt(1-zeta^2));
        p2 = exp(-2*zeta*w*h3);
        N = 2*pi/(w*h3*sqrt(1-zeta^2));
        
        syms s
        b = s^2 + p1*s + p2;
        lambda = solve(b,s);
        lambda = single(lambda)
        
        alpha = 1.1;
        pole1 = [lambda(1) lambda(2) real(lambda(1))^alpha 0.01+real(lambda(1)) 0.01+real(lambda(1))^alpha];              % Arbitrary pole locations
        pole2 = [0.1 0.05 0.97 0.98 0.7];
        pole3 = [lambda(1) lambda(2)];
        pole4 = [lambda(1) lambda(2)];
        
        K = place(discr_sys.a,discr_sys.b,pole2);
        K(4) = K(4);
        closedloop = discr_sys.a - discr_sys.b*K;
        K, eig(closedloop)
        
        
         %% LQR controller
        Q = diag([1 80000 0 0 1]);
        R = 10000;
        N = [0;0;0;0;0];
        
        [K,S,e] = dlqr(discr_sys.a,discr_sys.b,Q,R,N);
        closedloop = discr_sys.a - discr_sys.b*K;
        
        %% Observer
        alpha = 5;
        w = 4.6/(zeta*t_set)*alpha;       % settling time of 8 seconds
        p1 = -2*exp(-2*zeta*w*h)*cos(w*h*sqrt(1-zeta^2));
        p2 = exp(-2*zeta*w*h);
        syms s
        b = s^2 + p1*s + p2;
        lambda = solve(b,s);
        lambda = single(lambda);
        observer_poles = [lambda(1) lambda(2) real(lambda(1))^alpha -0.01+real(lambda(1))^alpha 0.01+real(lambda(1))^alpha];
        L = place(discr_sys.a',discr_sys.c',observer_poles)';
        
        %% Running the simulation
        x0 = [pi; 0; 0; 0; 0];
        states = Simulink.BlockDiagram.getInitialState(file)
        theta1 = pi; theta2 = 0;
        states.signals(2).values = theta1;          % Initial state theta1 for visualizer
        states.signals(4).values = theta2;          % Initial state theta2 for visualizer
        states.signals(8).values = [0 0 0 0 0];     % Observer around operating point
        
        t1 = 10;
        error = 1e-3;
        options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
        sim(file,t1,options);
        [t1,x,y,input] = sim(file,t1,options);
        theta1 = y(:,1); theta2 = y(:,2);
        return
end
