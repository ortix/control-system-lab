clearvars -except G_with_torque
%% Deriving a black box model from the downward position of the double pendulum and specifying a controller on this
%  The black box model is based on a constant torque output, where only
%  data is obtained from around the linearization point. Only estimate your
%  black box model for input to output2(theta2) to obtain a 5th order model
addpath('C:\Users\Dick\Documents\GitHub\control-system-lab\dick\Data\Constant black box')
addpath('..\')
% For use on TU Delft computer
addpath('H:\control-system-lab\dick\Data\Constant black box')

if ~exist('G_with_torque')      % Now around [pi; 0; 0; 0; 0]
    Pendulum_linid_blackbox;
end

%% Load data from counterclockwise (CCW) rotation
const_input_CCW = load('const_input1_CCW');
const_output_CCW = load('const_output1_CCW');

time = const_input_CCW.ans.Time;
h = diff(time(1:2));
U = const_input_CCW.ans.Data;
y = const_output_CCW.ans.Data;

% figure(1); 
% subplot(2,1,1);
% plot(time,y(:,1))
% ylabel('theta1')
% title('CCW rotation output data')
% subplot(2,1,2);
% plot(time,y(:,2))
% ylabel('theta2')

degr = 10;
rad  = degr*(pi/180);
op_point_da_index_CCW = find(y(:,1)<(pi+rad) & y(:,1)>(pi-rad));
for i = 1:length(op_point_da_index_CCW)
    index = op_point_da_index_CCW(i);
    U_CCW(i,1) = U(index);
    y_CCW(i,1) = y(index,1);
    y_CCW(i,2) = y(index,2);
    t_CCW(i,1) = i*h;
end
t_CCW = t_CCW-0.01;
%% Turning the data around the operating point in a fluent line

% For theta1 first
diff_index_CCW = find(diff(y_CCW(:,1))>0.1);
offset = 0;
ind = 1;
for i = 1:length(y_CCW)
     y_CCW(i,1) = y_CCW(i,1) + offset;
     if i == diff_index_CCW(ind)
       offset = y_CCW(i,1)-y_CCW(i+1,1);
       ind = ind + 1;
            if ind > length(diff_index_CCW)
                ind = 1;
            end
     end
end

% For theta2 secondly
diff_index_CCW = find(diff(y_CCW(:,2))<-0.03);
offset = 0;
ind = 1;

for i = 1:length(y_CCW)
     y_CCW(i,2) = y_CCW(i,2) + offset;
     if i == diff_index_CCW(ind)
       offset = y_CCW(i,2)-y_CCW(i+1,2);
       ind = ind + 1;
            if ind > length(diff_index_CCW)
                ind = 1;
            end
     end
end

% figure(2); 
% subplot(2,1,1);
% plot(t_CCW,y_CCW(:,1))
% ylabel('theta1')
% title('CCW rotation output data')
% subplot(2,1,2);
% plot(t_CCW,y_CCW(:,2))
% ylabel('theta2')

%% Estimation of OE model, OE because we have white measurement noise which is  filtered. Hence no Armax. OE also gives better fit

data1 = iddata(y_CCW(:,2),U_CCW,h);
nb = [2]; nf = [5]; nk = [1];
OE_theta2 = oe(data1,[nb nf nk]);  figure(3); compare(data1,OE_theta2);

% %% Putting the OE models to continuous time and a tf 
OE_tf_CCW = tf(d2c(OE_theta2)); 
OE_ss_CCW = ss(OE_tf_CCW);
% 
% % figure(2)
% % step(G_with_torque,OE_ss)

%% Create GBN signal
T = 5;       % length of experiment
h = 0.01;    % sampling interval
ts = 1;      % estimated settling time of the process
A = 0.1;     % amplitude of GBN
U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

figure(8)
subplot(2,1,1)
lsim(OE_ss_CCW,U(:,2),U(:,1))
title('Response of the black box model on a GBN input')
subplot(2,1,2)
lsim(G_with_torque(2),-U(:,2),U(:,1))
title('Response of the linearised model on a GBN input')

return
%% Load data from clockwise (CW) rotation
const_input_CW = load('const_input1_CW');
const_output_CW = load('const_output1_CW');

time = const_input_CW.ans.Time;
h = diff(time(1:2));
U = const_input_CW.ans.Data;
y = const_output_CW.ans.Data;

% figure(4); 
% subplot(2,1,1);
% plot(time,y(:,1))
% ylabel('theta1')
% title('CW rotation output data')
% subplot(2,1,2);
% plot(time,y(:,2))
% ylabel('theta2')

degr = 10;
rad  = degr*(pi/180);
op_point_da_index_CW = find(y(:,1)<(pi+rad) & y(:,1)>(pi-rad));
for i = 1:length(op_point_da_index_CW)
    index = op_point_da_index_CW(i);
    U_CW(i,1) = U(index);
    y_CW(i,1) = y(index,1);
    y_CW(i,2) = y(index,2);
    t_CW(i,1) = i*h;
end
t_CW = t_CW-0.01;
%% Turning the data around the operating point in a fluent line

% For theta1 first
diff_index_CW = find(diff(y_CW(:,1))<-0.1);
offset = 0;
ind = 1;
for i = 1:length(y_CW)
     y_CW(i,1) = y_CW(i,1) + offset;
     if i == diff_index_CW(ind)
       offset = y_CW(i,1)-y_CW(i+1,1);
       ind = ind + 1;
            if ind > length(diff_index_CW)
                ind = 1;
            end
     end
end

% For theta2 secondly
diff_index_CW = find(diff(y_CW(:,2))>0.1);
offset = 0;
ind = 1;

for i = 1:length(y_CW)
     y_CW(i,2) = y_CW(i,2) + offset;
     if i == diff_index_CW(ind)
       offset = y_CW(i,2)-y_CW(i+1,2);
       ind = ind + 1;
            if ind > length(diff_index_CW)
                ind = 1;
            end
     end
end

% figure(5);
% subplot(2,1,1);
% plot(t_CW,y_CW(:,1))
% ylabel('theta1')
% title('CW rotation output data')
% subplot(2,1,2);
% plot(t_CW,y_CW(:,2))
% ylabel('theta2')

%% Estimation of OE model, OE because we have white measurement noise which is  filtered. Hence no Armax. OE also gives better fit
data1 = iddata(y_CW(:,2),U_CW,h);
nb = [2]; nf = [5]; nk = [1];
OE_theta2 = oe(data1,[nb nf nk],'OutputName','Theta(2)'); % figure(6); compare(data1,OE_theta2);

%% Putting the OE models to continuous time and a tf 
OE_tf_CW = tf(d2c(OE_theta2)); 
% OE12_tf = tf([4.087 1564 4.376e5 4.478e7 2.463e7 2.833e9],[1 176.9 1.438e5 1.498e7 3.757e9 1.596e11 1.458e9])
OE_ss_CW = ss(OE_tf_CW);

% figure(2)
% step(G_with_torque,OE_ss)

% %% Merging the data and see what comes out 
% ind_CW = 1;
% ind_CCW = 1;
% state_CCW = 0;
% state_CW  = 1;
% % for i = 1:(length(diff_index_CCW)) % +length(op_point_da_index_CW))
% %     
% %     if i == diff_index_CCW(ind_CCW) && state_CCW == 0
% %         a = diff([diff_index_CCW(ind_CCW) diff_index_CCW(ind_CCW+1)])
% %         U_combined(i,1) = U_CCW(i);
% %         state_CCW = 1;
% %         state_CW =0;
% %     end
% %     
% %     if i == diff_index_CW(ind_CW) && state_CW == 0
% %         U_combined(i,1) = U_CW(i);
% %         state_CCW = 0; 
% %         state_CW = 1;
% %     end
% %     t_combined(i,1) = i*h;
% % end
% % 
% % figure(7)
% % plot(t_combined,U_combined)

%% Create GBN signal
T = 5;       % length of experiment
h = 0.01;    % sampling interval
ts = 1;      % estimated settling time of the process
A = 0.1;     % amplitude of GBN
U = [h*(0:T/h)' gbn(T,ts,A,h,1)];

figure(7)
subplot(2,1,1)
lsim(OE_ss_CW,U(:,2),U(:,1))
title('Response of the black box model on a GBN input')
subplot(2,1,2)
lsim(G_with_torque(2),-U(:,2),U(:,1))
title('Response of the linearised model on a GBN input')
