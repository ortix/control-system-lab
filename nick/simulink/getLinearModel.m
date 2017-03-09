function [sys] = getLinearModel(theta)
% clear all;

syms P1 P2 P3 g1 g2 theta1 theta2 theta1dot theta2dot theta1ddot theta2ddot b1 b2 Torque

M_theta = [P1+P2+2*P3*cos(theta2) P2+P3*cos(theta2)
             P2+P3*cos(theta2)          P2];
         
C_theta_thetadot = [b1-P3*theta2dot*sin(theta2)     -P3*(theta1dot + theta2dot)*sin(theta2)
                    P3*theta1dot                                b2];
                
G_theta = [-g1*sin(theta1)-g2*sin(theta1 + theta2); -g2*sin(theta1 + theta2)];

T = [1;0];

state = [theta1; theta2];
statedot = [theta1dot; theta2dot];
stateddot = [theta1ddot; theta2ddot];

%% Linearization with the torque in the system's matrix
syms P1 P2 P3 g1 g2 theta1 theta2 theta1dot theta2dot theta1ddot theta2ddot b1 b2 Torquedot u
tau_e = 0.03; k_m = 50;
T = -tau_e*Torquedot + k_m*u;
first_equation = M_theta(1,:)*stateddot + C_theta_thetadot(1,:)*statedot + G_theta(1) - T;
second_equation = M_theta(2,:)*stateddot + C_theta_thetadot(2,:)*statedot + G_theta(2);

theta2ddot_rewritten = solve(second_equation,theta2ddot);
theta1ddot_rewritten = solve(first_equation,theta1ddot);

first_equation_v2 = subs(first_equation,theta2ddot,theta2ddot_rewritten);
second_equation_v2 = subs(second_equation,theta1ddot,theta1ddot_rewritten);

theta1ddot_solved = solve(first_equation_v2,theta1ddot);
theta2ddot_solved = solve(second_equation_v2,theta2ddot);
% output = solve(first_equation,[theta1ddot theta2ddot])

jac_state = [theta1; theta2; theta1dot; theta2dot; Torquedot];
linearized_sys = [0 0 1 0 0; 0 0 0 1 0; jacobian(theta1ddot_solved,jac_state); jacobian(theta2ddot_solved,jac_state)];
jac_equation2 = jacobian(theta2ddot_solved,jac_state);

g = 9.81; 
l1 = 0.1;
l2 = 0.1;
m1 = 0.125;
m2 = 0.05;
c1 = -0.04;
c2 = 0.06;
I1 = 0.074;
I2 = 0.00012;
b1 = 4.8;
b2 = 0.0002;
k_m = 50;
tau_e = 0.03;

P1 = m1*c1^2+m2*l1^2+I1;
P2 = m2*c2^2+I2;
P3 = m2*l1*c2;
g1 = (m1*c1+m2*l1)*g;
g2 = m2*c2*g;

theta1 = theta(1); theta2 = theta(2); theta1dot = 0; theta2dot = 0;

theta1ddot_solved = vpa(subs(theta1ddot_solved),4);
theta2ddot_solved = vpa(subs(theta2ddot_solved),4);
linearized_sys_solved = double(subs(linearized_sys));
torquedot = [0 0 0 0 -(1/tau_e)];
A_torque = [linearized_sys_solved; torquedot];
% B = double(subs(inv(M_theta))); 
input_jacobian = jacobian([theta1ddot_solved;theta2ddot_solved],u);
B_torque = double([0;0;input_jacobian;k_m/tau_e]);
C = [1 0 0 0 0; 0 1 0 0 0];
D = [0;0];
states = { 'theta1' 'theta2' 'theta1_dot' 'theta2_dot' 'T_dot'};
inputs = {'torque'};
outputs = {'theta1'; 'theta2'};
sys = ss(A_torque,B_torque,C,D,'statename',states,'inputname',inputs,'outputname',outputs)


