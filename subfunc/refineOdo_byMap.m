function [ odo_refined, T_base_proj_refined, k_compass_refined_output, k_dist_refined_output ] = refineOdo_byMap( odo,...
    odoNum, mark, markNum, map, T_base_proj, k_compass_refined_input, k_dist_refined_input )
%REFINEODO_BYMAP Summary of this function goes here
%   Detailed explanation goes here

%% extract constraints
constraints = [];
for i = 1:(markNum-1)
    id1 = mark.id(i);
    id2 = mark.id(i+1);
    
%     if id1 ~= id2
%         continue;
%     end
    
    stamp1 = mark.stamp(i);
    stamp2 = mark.stamp(i+1);        
    T_odo_b1 = vec_matrix_2d([odo.x(stamp1);odo.y(stamp1);odo.theta(stamp1)]);
    T_odo_b2 = vec_matrix_2d([odo.x(stamp2);odo.y(stamp2);odo.theta(stamp2)]);
    T_b1_b2 = inv(T_odo_b1)*T_odo_b2;
    vec_b1_b2 = vec_matrix_2d(T_b1_b2);
    vec_c1_m1 = mark.vec2d(i,:).';
    vec_c2_m2 = mark.vec2d(i+1,:).';
    vec_w_m1 = map(id1+1).vec;
    vec_w_m2 = map(id2+1).vec;
    constraint_tmp.vec_b1_b2 = vec_b1_b2;
    constraint_tmp.vec_c1_m1 = vec_c1_m1;
    constraint_tmp.vec_c2_m2 = vec_c2_m2;
    constraint_tmp.vec_w_m1 = vec_w_m1;
    constraint_tmp.vec_w_m2 = vec_w_m2;
    constraint_tmp.stamp1 = stamp1;
    constraint_tmp.stamp2 = stamp2;
    constraint_tmp.id1 = id1;
    constraint_tmp.id2 = id2;
    constraints = [constraints constraint_tmp];   
end

%% calibrate the compass
[tmp, constraintsNum] = size(constraints);
a = []; b = [];
for i = 1:constraintsNum
    dtheta_odo = constraints(i).vec_b1_b2(3);    
    T_c1_m1 = vec_matrix_2d(constraints(i).vec_c1_m1);
    T_c2_m2 = vec_matrix_2d(constraints(i).vec_c2_m2);
    T_w_m1 = vec_matrix_2d(constraints(i).vec_w_m1);
    T_w_m2 = vec_matrix_2d(constraints(i).vec_w_m2);
    T_c1_c2 = T_c1_m1*inv(T_w_m1)*T_w_m2*inv(T_c2_m2);
    vec_c1_c2 = vec_matrix_2d(T_c1_c2);    
    dtheta_map = vec_c1_c2(3);
    a = [a;dtheta_odo];
    b = [b;dtheta_map];
end
k_theta = pinv(a)*b;
k_dist = 1;

%% refined the odometry with compass (k_theta) calibrated
odo_refined = odo;
for i = 1:(odoNum-1)
    x1 = odo.x(i);
    x2 = odo.x(i+1);
    y1 = odo.y(i);
    y2 = odo.y(i+1);
    theta1 = odo.theta(i);
    theta2 = odo.theta(i+1);    
    odo_refined.theta(i) = k_theta*theta1;
    odo_refined.theta(i+1) = k_theta*theta2;
    theta1_refined = odo_refined.theta(i);    
    dx = x2-x1;
    dy = y2-y1;
    R1 = vec_matrix_2d([0;0;theta1]);
    R1_refined = vec_matrix_2d([0;0;theta1_refined]);
    vec_tmp = k_dist*R1_refined*inv(R1)*[dx;dy;1];
    dx_refined = vec_tmp(1);
    dy_refined = vec_tmp(2);    
    odo_refined.x(i+1) = odo_refined.x(i)+dx_refined;
    odo_refined.y(i+1) = odo_refined.y(i)+dy_refined;
end

%% renew constraints with refined odometry
for i = 1:constraintsNum
    stamp1 = constraints(i).stamp1;
    stamp2 = constraints(i).stamp2;
    T_odo_b1 = vec_matrix_2d([odo_refined.x(stamp1);odo_refined.y(stamp1);odo_refined.theta(stamp1)]);
    T_odo_b2 = vec_matrix_2d([odo_refined.x(stamp2);odo_refined.y(stamp2);odo_refined.theta(stamp2)]);
    T_b1_b2 = inv(T_odo_b1)*T_odo_b2;
    vec_b1_b2 = vec_matrix_2d(T_b1_b2);
    constraints(i).vec_b1_b2 = vec_b1_b2;
end

%% calibrate k_dist and camera extrinsics
vec_b_c = vec_matrix_2d(T_base_proj);
vec_alpha_init = [vec_b_c;k_dist];
% vec_alpha = fminunc(@(x)calib2d_fun_withmap_dest(x, constraints),vec_alpha_init);
vec_alpha = fmincon(@(x)calib2d_fun_withmap_dest(x, constraints),vec_alpha_init,[],[],[],[],[],[],@(x)calib2d_fun_withmap_constr(x, vec_b_c(3)));
T_base_proj_refined = vec_matrix_2d(vec_alpha(1:3));
k_dist = vec_alpha(4);

%% refine odometry with k_dist
odo_refined = odo;
for i = 1:(odoNum-1)
    x1 = odo.x(i);
    x2 = odo.x(i+1);
    y1 = odo.y(i);
    y2 = odo.y(i+1);
    theta1 = odo.theta(i);
    theta2 = odo.theta(i+1);    
    odo_refined.theta(i) = k_theta*theta1;
    odo_refined.theta(i+1) = k_theta*theta2;
    theta1_refined = odo_refined.theta(i);    
    dx = x2-x1;
    dy = y2-y1;
    R1 = vec_matrix_2d([0;0;theta1]);
    R1_refined = vec_matrix_2d([0;0;theta1_refined]);
    vec_tmp = k_dist*R1_refined*inv(R1)*[dx;dy;1];
    dx_refined = vec_tmp(1);
    dy_refined = vec_tmp(2);    
    odo_refined.x(i+1) = odo_refined.x(i)+dx_refined;
    odo_refined.y(i+1) = odo_refined.y(i)+dy_refined;
end

%% record k
k_compass_refined_output = k_compass_refined_input * k_theta;
k_dist_refined_output = k_dist_refined_input * k_dist;
end

