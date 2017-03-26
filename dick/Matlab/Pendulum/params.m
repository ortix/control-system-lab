% %% Old parameters
% g = 9.81; 
% l1 = 0.1;
% l2 = 0.1;
% m1 = 0.125;
% m2 = 0.05;
% c1 = 0.04; % Keeping it negative results in no motion at all, -0.04 or + 0.04?
% c2 = 0.06;
% I1 = 0.074;
% I2 = 0.00012;
% b1 = 4.8;
% b2 = 0.0002;
% k_m = 50;
% tau_e = 0.03;
% 
% P1 = m1*c1^2+m2*l1^2+I1;
% P2 = m2*c2^2+ I2;
% P3 = m2*l1*c2;
% g1 = (m1*c1+m2*l1)*g;
% g2 = m2*c2*g;
% 
% h = 0.01;

%% New parameters after parameter estimation
g = 9.81; 
l1 = 0.1;
l2 = 0.1;
m1 = 0.125;
m2 = 0.05;
c1 = -0.04; % Keeping it negative results in no motion at all, -0.04 or + 0.04?
c2 = 0.06;
I1 = 0.0029;
I2 = 1e-5*7.8856;
b1 = 5.8039;
b2 = 3.1250e-5;
k_m = 41.0587;
tau_e = 0.0312;

P1 = m1*c1^2+m2*l1^2+I1;
P2 = m2*c2^2+ I2;
P3 = m2*l1*c2;
g1 = (m1*c1+m2*l1)*g;
g2 = m2*c2*g;

h = 0.01;