clear all; clc;

syms P1 P2 P3 g1 g2 theta1 theta2 theta1dot theta2dot theta1ddot theta2ddot b1 b2 T

M_theta = [P1+P2+2*P3*cos(theta2) P2+P3*cos(theta2)
             P2+P3*cos(theta2)          P2];
         
C_theta_thetadot = [b1-P3*theta2dot*sin(theta2)     -P3*(theta1dot + theta2dot)*sin(theta2)
                    P3*theta1dot                                b2];
                
G_theta = [-g1*sin(theta1)-g2*sin(theta1 + theta2); -g2*sin(theta1 + theta2)];

state = [theta1; theta2];
statedot = [theta1dot; theta2dot];
stateddot = [theta1ddot; theta2ddot];

first_equation = M_theta(1,:)*stateddot + C_theta_thetadot(1,:)*statedot + G_theta(1);
second_equation = M_theta(2,:)*stateddot + C_theta_thetadot(2,:)*statedot + G_theta(2);

theta2ddot_rewritten = solve(second_equation,theta2ddot);
theta1ddot_rewritten = solve(first_equation,theta1ddot);

first_equation_v2 = subs(first_equation,theta2ddot,theta2ddot_rewritten);
second_equation_v2 = subs(second_equation,theta1ddot,theta1ddot_rewritten);

theta1ddot_solved = solve(first_equation_v2,theta1ddot)
theta2ddot_solved = solve(second_equation_v2,theta2ddot)
% output = solve(first_equation,[theta1ddot theta2ddot])

jac_state = [theta1; theta2; theta1dot; theta2dot];
linearized_sys = [jacobian(theta1ddot_solved,jac_state); jacobian(theta2ddot_solved,jac_state)];
jac_equation2 = jacobian(theta2ddot_solved,jac_state);
% A = 

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

theta1ddot_solved = vpa(subs(theta1ddot_solved),4)
theta2ddot_solved = vpa(subs(theta2ddot_solved),4)
