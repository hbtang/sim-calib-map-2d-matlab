clear;
% syms x_w_m1 y_w_m1 theta_w_m1
% syms x_w_m2 y_w_m2 theta_w_m2
syms x_b1_b2 y_b1_b2 theta_b1_b2
syms x_c1_m1 y_c1_m1 theta_c1_m1
syms x_c2_m2 y_c2_m2 theta_c2_m2
syms dx dy kd
syms theta_c_b

T_c1_m1 = [cos(theta_c1_m1) -sin(theta_c1_m1) x_c1_m1;...
    sin(theta_c1_m1) cos(theta_c1_m1) y_c1_m1;...
    0 0 1];
T_c2_m2 = [cos(theta_c2_m2) -sin(theta_c2_m2) x_c2_m2;...
    sin(theta_c2_m2) cos(theta_c2_m2) y_c2_m2;...
    0 0 1];
T_b1_b2 = [cos(theta_b1_b2) -sin(theta_b1_b2) kd*x_b1_b2;...
    sin(theta_b1_b2) cos(theta_b1_b2) kd*y_b1_b2;...
    0 0 1];
T_b_c = [cos(theta_c_b) -sin(theta_c_b) dx;...
    sin(theta_c_b) cos(theta_c_b) dy;...
    0 0 1];

T = inv(T_c1_m1)*inv(T_b_c)*T_b1_b2*T_b_c*T_c2_m2;
T = simplify(T);

x_m1_m2 = T(1,3)
y_m1_m2 = T(2,3)