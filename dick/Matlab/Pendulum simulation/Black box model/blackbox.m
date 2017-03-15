% clear all; 
%% Deriving a black box model from the downward position of the double pendulum and specifying a controller on this
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\GBN')
addpath('..\')

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
n = 2;
fs = 100;                     % sampling frequency
fcut = 10;                    % [Hz]
[b,a] = butter(n,fcut/fs);  % butterworth low pass filter
y_filt = filtfilt(b,a,y);
% y_filt = y;
%% Estimation of OE model, OE because we have white measurement noise which is  filtered. Hence no Armax. OE also gives better fit
figure(1);
data1 = iddata(y_filt(:,:),U,h);
nb = [1;4]; nf = [5;5]; nk = [1;1];
OE11 = oe(data1,[nb nf nk]); subplot(2,1,1); compare(data1,OE11);

%% Putting the OE models to continuous time and a tf 
OE_tf = tf(d2c(OE11)); 
% OE12_tf = tf([4.087 1564 4.376e5 4.478e7 2.463e7 2.833e9],[1 176.9 1.438e5 1.498e7 3.757e9 1.596e11 1.458e9])
OE_ss = ss(OE_tf);

figure(2)
step(G_with_torque,OE_ss)

%% For running the simulation
x1 = 0.1/23.74; x2 = 0/10.55;

return
%% Load the second data set for validation 
GBN_input = load('GBN_input2');
GBN_output = load('GBN_output2');

time = GBN_input.ans.Time;
h = diff(time(1:2));
U2 = GBN_input.ans.Data;
y2 = GBN_output.ans.Data;

%% Filtering output data
n = 2;
fs = 100;                     % sampling frequency
fcut = 10;                    % [Hz]
[b,a] = butter(n,fcut/fs);  % butterworth low pass filter
y2_filt = filtfilt(b,a,y2);
% y_filt = y;

%% Validate the model
data2 = iddata(y2_filt(:,:),U2,h);
figure(3); subplot(2,1,1); compare(data2,OE11);
