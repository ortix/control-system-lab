clc;clear;close all
load yhat_kalman;
load yhat_observer;
%% Plot graphs
disp('Creating pretty graphs')
% Theta1
figure('units','normalized','position',[0.2 0.2 0.4 0.4]);
subplot(2,2,1);hold on;grid on;
plot(yhat_observer.time,yhat_observer.Data(:,1),':k','linewidth',2)
plot(yhat_observer.time,yhat_observer.Data(:,3),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 1 Observer')
ylim([-0.03 0.03]);xlim([0 2])
ylabel('amplitude [rad]');xlabel('time [s]')

legend('unfiltered','filtered','Location','south')

subplot(2,2,2);hold on;grid on;
plot(yhat_kalman.time,yhat_kalman.Data(:,1),':k','linewidth',2)
plot(yhat_kalman.time,yhat_kalman.Data(:,3),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 1 Kalman')
ylabel('amplitude [rad]');xlabel('time [s]')
ylim([-0.03 0.03]);xlim([0 2])
legend('unfiltered','filtered','Location','south')

%Theta2
subplot(2,2,3);hold on; grid on;
plot(yhat_observer.time,yhat_observer.Data(:,2),':k','linewidth',2)
plot(yhat_observer.time,yhat_observer.Data(:,4),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 2 Observer')
ylabel('amplitude [rad]');xlabel('time [s]')
ylim([-0.1 0.1]);xlim([0 2])
legend('unfiltered','filtered','Location','south')

subplot(2,2,4);hold on; grid on;
plot(yhat_kalman.time,yhat_kalman.Data(:,2),':k','linewidth',2)
plot(yhat_kalman.time,yhat_kalman.Data(:,4),'-','linewidth',2,'color',[0.5 0.5 0.5])
title('Theta 2 Kalman')
ylabel('amplitude [rad]');xlabel('time [s]')
ylim([-0.1 0.1]);xlim([0 2])
legend('unfiltered','filtered','Location','south')
saveas(gcf,'../presentation/img/observer_vs_kalman','png');