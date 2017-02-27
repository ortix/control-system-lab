g = 9.81; 
l1 = 0.1;
l2 = 0.1;
m1 = 0.125;
m2 = 0.05;
c1 = 0.04; % Keeping it negative results in no motion at all, -0.04 or + 0.04?
c2 = 0.06;
I1 = 0.074;
I2 = 0.00012;
b1 = 4.8;
b2 = 0.0002;
k_m = 50;
tau_e = 0.03;

P1 = m1*c1^2+m2*l1^2+I1;
P2 = m2*c2^2+ I2;
P3 = m2*l1*c2;
g1 = (m1*c1+m2*l1)*g;
g2 = m2*c2*g;

h = 0.01;