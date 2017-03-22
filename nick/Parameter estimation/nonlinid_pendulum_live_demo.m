clear all; clc
%% Nonlinear identification of parameters of the pendulum model
% First an approximation of the parameters of the first link is made.
% Secondly the found parameters are used to find the parameters of the
% second link

%% Generate a crested multisine input to excite the system with
addpath('Basic plant')
calib;
hwinit;

generate_step;

sim('rpend_dick',20)

        time = U(:,1);
        U = U(:,2);
        % Process data from theta1
        n = find(diff(y(:,1)>4));
        y(n(1)+1:end,1) = y(n(1)+1:end,1) - 2*pi; 
        % Process data from theta2
        n = find(diff(y(:,2)>4));
        y(n(2)+1:end,2) = y(n(2)+1:end,2) + 2*pi;
        y = y(1:length(time),:);
% U should be timeseries
U = [time U];
% figure(2); p1 = plot(time,y(:,1)); hold on; p2 = plot(time,y(:,2)); 
% xlabel('Time [sec]')
% ylabel('angle [rad]')
% title('Output of the real system and the simulation model for the same input')
% 
%% Non linear parameter estimation
% When estimating parameters of the first link: 
% initial guess: [I1 b1 I2 b2 k_m tau_e]
parameters_initial_guess = [0.074 4.8 1e-4*0.8857 1e-4*0.5459 50 0.3].';   % initial guess

% % Calculate damping term b2 from data
% [x_max1, ind1] = max(y(1:50,2));
% [x_max2, ind2] = max(y(50:end,2));
% t_cycle = time(ind2+49)-time(ind1);
% w_n = (2*pi)/t_cycle;
% T_s = 25.7;
% zeta = 4.6/(T_s*w_n);
% b2 = 2*zeta*w_n;


% When estimating parameters of the first link:
% initial guess: [I2 b2]
% parameters_initial_guess = [0.00012 0.0002].';   % initial guess

k_m = 50;
tau_e = 0.3;
% Fill in initial guess. Link 1: x0 = [pi 0 0 0 0]
                       % Link 2: x0 = [0.5*pi 0 0 0 0]
x0 =[pi 0 0 0 0];

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % design input excitation signal
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T = 10;           % length of experiment
% h = 0.01;         % sampling interval
% ts = 0.1;         % estimated settling time of the process
% A = 0.1;          % amplitude of GBN
% U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% assignin('base','k_m',k_m)
% sim('Pendulum_model_nlsysid',20);     % ouput data available in y
% 
% figure(2); p3 = plot(U(:,1),y(:,1)); p4 = plot(U(:,1),y(:,2));
% legend([p1,p2,p3,p4],'theta1 real system','theta2 real system','theta1 simulation model','theta2 simulation model')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lb = [0 0 0 0 0 0]; up = [0.2 10 1e-2 1e-2 50 1];
OPT = optimoptions(@lsqnonlin,'MaxIter',25);
[xhat,fval]= lsqnonlin('costfun_pendulum',parameters_initial_guess,lb,up,OPT,U,y);
[parameters_initial_guess xhat], fval
figure(1); % legend('Real data','Fitted data')
title('Parameter estimation of link 1')
xlabel('time [sec]')
ylabel('angle [rad]')