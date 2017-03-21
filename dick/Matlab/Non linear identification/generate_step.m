%% Generate a step or impulse input for the system after 1 second in simulation 

%% Step
h = 0.01;
Tend = 20;
t = (0:h:Tend-h).';
step_input = zeros(length(t),1);

% step
for i = 1:length(t)
    if t(i) < 1
        step_input(i) = 1;
    end
    
    if t(i) > 10 && t(i) < 11
        step_input(i) = 1;
    end
end
% figure(1)
% plot(t,step_input)
U = [t step_input]

% %%  Impulse
% impulse_input = zeros(length(t),1);
% for i = 1:200:length(t)
%     impulse_input(i) = 1;
%     impulse_input(i+100) = -1;
% end
% 
% 
% figure(1)
% plot(t,impulse_input)
% U = [t impulse_input]
% 
% %% Swing input vector
% Tend = 40;
% t = (0:h:Tend-h).';
% U = [t zeros(length(t),1)];

%% Generate constant input
% h = 0.01;
% Tend = 50;
% t = (0:h:Tend-h).';
% amplitude = -0.3;
% const_input = repmat(amplitude,[length(t),1]);
% 
% figure(1)
% plot(t,const_input)
% U = [t const_input]