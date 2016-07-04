function [ res ] = calib2d_fun_withmap_dest( x, constraints)
%CALIB2D_FUN_WITHMAP_DEST Summary of this function goes here
%   Detailed explanation goes here

std_x = 20; % 20 mm
std_y = 20;
std_theta = 1*pi/180; % 3 degree
Conv_opt = diag([std_x^2;std_y^2;std_theta^2]);
Inf_opt = inv(Conv_opt);

[tmp, constraintsNum] = size(constraints);
res = 0;
for i = 1:constraintsNum
    vec_b_c = x(1:3);
    T_b_c = vec_matrix_2d(vec_b_c);
    k_dist = x(4);    
    vec_b1_b2 = constraints(i).vec_b1_b2;
    vec_c1_m1 = constraints(i).vec_c1_m1;
    vec_c2_m2 = constraints(i).vec_c2_m2;
    vec_w_m1 = constraints(i).vec_w_m1;
    vec_w_m2 = constraints(i).vec_w_m2;
    vec_b1_b2 = [k_dist*vec_b1_b2(1:2);vec_b1_b2(3)];
    
    T_b1_b2 = vec_matrix_2d(vec_b1_b2);
    T_c1_m1 = vec_matrix_2d(vec_c1_m1);
    T_c2_m2 = vec_matrix_2d(vec_c2_m2);
    T_w_m1 = vec_matrix_2d(vec_w_m1);
    T_w_m2 = vec_matrix_2d(vec_w_m2);
    
    T_m1_m2_map = inv(T_w_m1)*T_w_m2;
    T_m1_m2_odo = inv(T_c1_m1)*inv(T_b_c)*T_b1_b2*T_b_c*T_c2_m2;
    
    dT = T_m1_m2_map*inv(T_m1_m2_odo);
    dvec = vec_matrix_2d(dT);
      
    res = res + dvec.'*Inf_opt*dvec;
end

end

