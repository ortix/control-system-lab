function e = costfun_pendulum(x,U,y)
% cost function for nonlinear parameter tuning
% When estimating the parameters for the first link:
% x = [I2 b2]

%% For link 1
% % Assign x to simulink
% assignin('base','input_costfun',x);
% assignin('base','k_m_hat',x(5));
% assignin('base','tau_e_hat',x(6));

%% For link 2
% Assign x to simulink
assignin('base','input_costfun',x);

% asses simulation
% Which model is evaluated? Link 2 --> pendulum_nlmodel_link2 etc
error = 1e-3;
options = simset('SrcWorkspace','current','AbsTol',error,'RelTol',error,'MaxStep',0.01);
[tm,xm,ym]= sim('Pendulum_nlmodel_link2',U(:,1),[],U);  % simulate nonlinear model

%% For link 1
% e = y(:,:)-ym(:,:);                               % residual (error)

%% For link 2
e = y(1:1500,2)-ym(1:1500,2);

% you can comment the below line to speed up
%% For plotting parameter estimation of link 1
% figure(1); stairs(tm,y(:,:),tm, ym(:,:));           % intermediate fit
% drawnow();

%% For plotting parameter estimation of link 2
figure(1); stairs(tm,y(:,2),'k'); hold; stairs(tm(1:3:end), ym(1:3:end,2),'k:','Linewidth',2);           % intermediate fit
% figure(1); stairs(tm,[y(:,2) ym(:,2)]); 
drawnow();
%pause