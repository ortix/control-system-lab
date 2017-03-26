% clear all; 
%% Deriving a black box model from the downward position of the double pendulum only for theta2
% Trying to do it both was futile
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\GBN')
addpath('..\')
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Matlab\Non linear identification')

if ~exist('G_with_torque')      % Now around [pi; 0; 0; 0; 0]
    Pendulum_linid_blackbox;
end

GBN_input = load('GBN_input1');
GBN_output = load('GBN_output1');

time = GBN_input.ans.Time;
h = diff(time(1:2));
U = GBN_input.ans.Data;
y = GBN_output.ans.Data;

%% Filtering output data
% n = 2;
% fs = 100;                     % sampling frequency
% fcut = 10;                    % [Hz]
% [b,a] = butter(n,fcut/fs);    % butterworth low pass filter
% y_filt = filtfilt(b,a,y);
% % y_filt = y;
%% Estimation of OE model, OE because we have white measurement noise which is  filtered. Hence no Armax. OE also gives better fit
data1 = iddata(y(:,2),U,h);
nb = [4]; nf = [5]; nk = [1];
OE11 = oe(data1,[nb nf nk], 'OutputName', 'Theta(2)'); figure(1); plot_figure; compare(data1,OE11,'--');

%% Putting the OE models to continuous time and a tf 
OE_tf = tf(d2c(OE11)); 
% OE12_tf = tf([4.087 1564 4.376e5 4.478e7 2.463e7 2.833e9],[1 176.9 1.438e5 1.498e7 3.757e9 1.596e11 1.458e9])
OE_ss = ss(OE_tf);

%% Generate input signal (step)
generate_step;

figure(2); 
plot_figure;
subplot(2,1,1);
lsim(OE_ss,U(:,2),U(:,1))
title('Response of the black box model on a step input')
subplot(2,1,2);
lsim(G_with_torque,U(:,2),U(:,1))
title('Response of the linearised system on a step input')
