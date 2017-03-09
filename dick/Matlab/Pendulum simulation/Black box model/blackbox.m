clear all; 
%% Deriving a black box model from the downward position of the double pendulum and specifying a controller on this
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\GBN')
GBN_input = load('GBN_input1');
GBN_output = load('GBN_output1');

time = GBN_input.ans.Time;
h = diff(time(1:2));
U = GBN_input.ans.Data;
y = GBN_output.ans.Data;

figure(1)
subplot(2,1,1)
plot(time,U); 
subplot(2,1,2)
plot(time,y);

figure(1); clf;
data11 = iddata(y(:,1),U,h);
nb = [1]; nf = [5]; nk = [1];
M11 = oe(data11,[nb nf nk]); subplot(2,1,1); compare(data11,M11);

data12 = iddata(y(:,2),U,h);
nb =[4]; nf = [5]; nk = [1];
M12 = oe(data12,[nb nf nk]); subplot(2,1,2); compare(data12,M12);

