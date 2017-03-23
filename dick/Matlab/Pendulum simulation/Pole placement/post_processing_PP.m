%% Processing the data of the pole placement controller
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\Pole placement')

% load in the data
pp_input = load('input_PP_dist1');
pp_output = load('output_PP_dist1');

% Measurement was 15 seconds, to obtain the correct data only take the last
% 10 seconds
time = pp_input.ans.Time;
h = diff(time(1:2));
U = pp_input.ans.Data;
y = pp_output.ans.Data;

time = time(1:1001);
U = U(501:1501,1);
y = y(501:1501,:);

figure(1)
plot(time, U,'k');
xlabel('Time [sec]')
ylabel('Amplitude')
title('Input signal') 
plot_figure_minipage;

figure(2)
plot(time,y(:,1),'k-','Linewidth',0.1); hold; plot(time,y(:,2),'k--','Linewidth',2); hold
xlabel('Time [sec]')
ylabel('Angle [rad]')
legend({'\theta_1','\theta_2'},'FontSize',16)
title('Output signal')
plot_figure_minipage;
