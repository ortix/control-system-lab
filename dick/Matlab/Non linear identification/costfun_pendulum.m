function e = costfun_pendulum(x,U,y)
% cost function for nonlinear parameter tuning
% When estimating the parameters for the first link:
% x = [I2 b2]

% Assign x to simulink
assignin('base','input_costfun',x);
assignin('base','k_m_hat',50);
assignin('base','tau_e_hat',0.03);

% asses simulation
% Which model is evaluated? Link 2 --> pendulum_nlmodel_link2 etc
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
[tm,xm,ym]= sim('Pendulum_nlmodel_link2',U(:,1),[],U);  % simulate nonlinear model

e = y(1:1500,2)-ym(1:1500,2);                               % residual (error)

% you can comment the below line to speed up
figure(1); stairs(tm,[y ym]);           % intermediate fit
drawnow();
%pause