function statedot = pendulum_solver(t,state)
theta = state(1:2);
theta_dot = state(3:4);

g = 9.81; 
l1 = 0.1;
l2 = 0.1;
m1 = 0.125;
m2 = 0.05;
c1 = -0.04; % Keeping it negative results in no motion at all
c2 = 0.06;
I1 = 0.074;
I2 = 0.00012;
b1 = 4.8;
b2 = 0.0002;

P1 = m1*c1^2+m2*l1^2+I1;
P2 = m2*c2^2+I2;
P3 = m2*l1*c2;
g1 = (m1*c1+m2*l1)*g;
g2 = m2*c2*g;

M = [(P1 + P2 + 2*P3*cos(theta(2)))               (P2 + P3*cos(theta(2))) ;
     (P2 + P3*cos(theta(2)))                       P2                   ];
 
 
C = [(b1 - P3*theta_dot(2)*sin(theta(2)))           (-P3*(theta_dot(1)+theta_dot(2))*sin(theta(2))) ;
     (P3*theta_dot(1)*sin(theta(2)))                 b2                                      ];
     
G = [(-g1*sin(theta(1))-g2*sin(theta(1) + theta(2))) ;
      -g2*sin(theta(1)+theta(2))                   ];
  
T = 0;  
thetaddot = M\[T;0] - M\C*theta_dot - M\G;
statedot = [theta_dot; thetaddot];
end


