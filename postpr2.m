clc;clear;close all
load yhat_plant;
%% Plot graphs
disp('Creating pretty graphs')
% Theta1
figure('units','normalized','position',[0.2 0.2 0.4 0.4]);
subplot(2,2,1);hold on;grid on;
plot(yhat_plant.time,yhat_plant.Data(:,1),':k','linewidth',2)
plot(yhat_plant.time,yhat_plant.Data(:,3),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 1 Plant')
xlim([0 10])
ylabel('amplitude [rad]');xlabel('time [s]')
legend('unfiltered','filtered','Location','southeast')

subplot(2,2,2);hold on;grid on;
plot(yhat_plant.time,yhat_plant.Data(:,2),':k','linewidth',2)
plot(yhat_plant.time,yhat_plant.Data(:,4),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 2 Plant')
xlim([0 10])
ylabel('amplitude [rad]');xlabel('time [s]')
legend('unfiltered','filtered','Location','northeast')

subplot(2,2,3);hold on;grid on;
plot(yhat_plant.time,yhat_plant.Data(:,1),':k','linewidth',2)
plot(yhat_plant.time,yhat_plant.Data(:,3),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 1 Plant')
ylim([-0.2 0.9]);xlim([5 7])
ylabel('amplitude [rad]');xlabel('time [s]')
legend('unfiltered','filtered','Location','northwest')

subplot(2,2,4);hold on;grid on;
plot(yhat_plant.time,yhat_plant.Data(:,2),':k','linewidth',2)
plot(yhat_plant.time,yhat_plant.Data(:,4),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 2 Plant')
ylim([-0.9 0.3]);xlim([5 7])
ylabel('amplitude [rad]');xlabel('time [s]')
legend('unfiltered','filtered','Location','southwest')

saveas(gcf,'presentation/img/observer_vs_kalman','png');