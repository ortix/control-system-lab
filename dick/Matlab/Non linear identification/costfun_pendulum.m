function e = costfun_pendulum(x,U,y)
% cost function for nonlinear parameter tuning
% When estimating the parameters for the first link:
% x = [b1 k_m tau_e]

% Assign x to simulink
assignin('base','input_costfun',[x(1) x(2)]);
assignin('base','k_m_hat',x(3));
assignin('base','tau_e_hat',x(4));

x

% asses simulation
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
[tm,xm,ym]= sim('Pendulum_nlmodel',U(:,1),[],U);  % simulate nonlinear model

e = y-ym;                               % residual (error)

% you can comment the below line to speed up
figure(1); stairs(tm,[y ym]);           % intermediate fit
%pause