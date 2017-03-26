function e = costfun(x,U,y)
% cost function for nonlinear parameter tuning

assignin('base','bhat',x);              % assign bhat in workspace
[tm,xm,ym]=sim('nlmodel',U(:,1),[],U);  % simulate nonlinear model

e = y-ym;                               % residual (error)

% you can comment the below line to speed up
figure(1); stairs(tm,[y ym]);           % intermediate fit
%pause