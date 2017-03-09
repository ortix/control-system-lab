clear all; 
%% Deriving a black box model from the downward position of the double pendulum and specifying a controller on this
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\GBN')
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

f = (1:length(time))./time(end); % Create a frequency vector

% figure(3)
% Y = fft(y);
% Y_filt = fft(y_filt)
% loglog(f,abs(Y))
% figure(4)
% loglog(f,abs(Y_filt))

figure(1)
subplot(2,1,1)
plot(time,U); 
subplot(2,1,2)
plot(time,y);

figure(1); clf;
data11 = iddata(y_filt(:,1),U,h);
nb = [1]; nf = [5]; nk = [1];
OE11 = oe(data11,[nb nf nk]); subplot(2,1,1); compare(data11,OE11);

data12 = iddata(y_filt(:,2),U,h);
nb =[4]; nf = [5]; nk = [1];
OE12 = oe(data12,[nb nf nk]); subplot(2,1,2); compare(data12,OE12);


%% Armax model is not good with filtered data!
% figure(2); clf;
% nb = 1; na = 5; nc = 1; nk = 1;
% A11 = armax(data11,[na nb nc nk]); subplot(2,1,1); compare(data11,A11);
% 
% nb = 4; na = 5; nc = 5; nk = 1;
% A12 = armax(data12,[na nb nc nk]); subplot(2,1,2); compare(data12,A12);
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

data11 = iddata(y_filt(:,1),U,h);

data12 = iddata(y_filt(:,2),U,h);
