function [ T_b_c, k_dist, k_theta, Alpha, odo_refine ] =Proc_RefineParam( Alpha, odo_raw, k_dist, k_theta, sz_odo, mark, sz_mark, T_b_c, ...
        hash_Alpha_odoIdx, hash_Alpha_markIdx )
%PROC_REFINEPARAM Summary of this function goes here

%% set scale on map
idx_tmp = find(hash_Alpha_markIdx(:,2) == 0);
idx_m0 = hash_Alpha_markIdx(idx_tmp,1);
idx_tmp = find(hash_Alpha_markIdx(:,2) == 3);
idx_m3 = hash_Alpha_markIdx(idx_tmp,1);
dist_m0m3_ref = 12000;
vec_m0 = Alpha(idx_m0*3-2: idx_m0*3);
vec_m3 = Alpha(idx_m3*3-2: idx_m3*3);
dist_m0m3 = norm(vec_m3(1:2)-vec_m0(1:2));
k_m0m3 = dist_m0m3_ref/dist_m0m3;

Alpha(1:3:end-2) = Alpha(1:3:end-2)*k_m0m3;
Alpha(2:3:end-1) = Alpha(2:3:end-1)*k_m0m3;

k_dist = k_dist*k_m0m3;
[ odo_refine ] = Proc_RefineOdo( odo_raw, k_dist, k_theta);

%% refine parameters T_b_c
A = [];
b = [];
A_theta = [];
b_theta = [];

for j = 1:sz_mark-1
    id_m1 = mark.id(j);
    id_m2 = mark.id(j+1);
    
%     if id_m1 ~= id_m2
%         continue;
%     end
    
    vec_c1_m1 = mark.vec2d(j,:);
    vec_c2_m2 = mark.vec2d(j+1,:);
    
    idx_alpha_b1 = mark.odoIdx(j);
    idx_alpha_b2 = mark.odoIdx(j+1);
    
    vec_w_b1 = [odo_refine.x(idx_alpha_b1);odo_refine.y(idx_alpha_b1);odo_refine.theta(idx_alpha_b1)];
    vec_w_b2 = [odo_refine.x(idx_alpha_b2);odo_refine.y(idx_alpha_b2);odo_refine.theta(idx_alpha_b2)];
    %     vec_w_b1 = Alpha(idx_alpha_b1*3-2:idx_alpha_b1*3);
    %     vec_w_b2 = Alpha(idx_alpha_b2*3-2:idx_alpha_b2*3);
    
    
    idx_tmp = find(hash_Alpha_markIdx(:,2) == mark.id(j));
    idx_alpha_m1 = hash_Alpha_markIdx(idx_tmp,1);
    vec_w_m1 = Alpha(idx_alpha_m1*3-2:idx_alpha_m1*3);
    
    idx_tmp = find(hash_Alpha_markIdx(:,2) == mark.id(j+1));
    idx_alpha_m2 = hash_Alpha_markIdx(idx_tmp,1);
    vec_w_m2 = Alpha(idx_alpha_m2*3-2:idx_alpha_m2*3);
    
    T_w_b1 = Trans_Pose_to_Mat(vec_w_b1);
    T_w_m1 = Trans_Pose_to_Mat(vec_w_m1);
    T_c1_m1 = Trans_Pose_to_Mat(vec_c1_m1);
    
    T_w_b2 = Trans_Pose_to_Mat(vec_w_b2);
    T_w_m2 = Trans_Pose_to_Mat(vec_w_m2);
    T_c2_m2 = Trans_Pose_to_Mat(vec_c2_m2);
    
    vec_b_c = Trans_Mat_to_Pose(T_b_c);
    
    T_b1_b2 = inv(T_w_b1)*T_w_b2;
    vec_b1_b2 = Trans_Mat_to_Pose(T_b1_b2);
    
    T_m1_m2 = inv(T_w_m1)*T_w_m2;
    vec_m1_m2 = Trans_Mat_to_Pose(T_m1_m2);
    
    x_m1_m2 = vec_m1_m2(1); y_m1_m2 = vec_m1_m2(2); theta_m1_m2 = vec_m1_m2(3);
    x_b1_b2 = vec_b1_b2(1); y_b1_b2 = vec_b1_b2(2); theta_b1_b2 = vec_b1_b2(3);
    x_c1_m1 = vec_c1_m1(1); y_c1_m1 = vec_c1_m1(2); theta_c1_m1 = vec_c1_m1(3);
    x_c2_m2 = vec_c2_m2(1); y_c2_m2 = vec_c2_m2(2); theta_c2_m2 = vec_c2_m2(3);
    theta_c_b = vec_b_c(3);
    
    A_tmp = zeros(2,3);
    b_tmp = zeros(2,1);
    
    A_tmp(1,1) = cos(theta_c_b - theta_b1_b2 + theta_c1_m1) - cos(theta_c_b + theta_c1_m1);
    A_tmp(1,2) = sin(theta_c_b - theta_b1_b2 + theta_c1_m1) - sin(theta_c_b + theta_c1_m1);
    A_tmp(1,3) = x_b1_b2*cos(theta_c_b + theta_c1_m1) + y_b1_b2*sin(theta_c_b + theta_c1_m1);
    b_tmp(1) = x_m1_m2 + x_c1_m1*cos(theta_c1_m1) + y_c1_m1*sin(theta_c1_m1) - x_c2_m2*cos(theta_b1_b2 - theta_c1_m1) + y_c2_m2*sin(theta_b1_b2 - theta_c1_m1);
    
    A_tmp(2,1) = - sin(theta_c_b - theta_b1_b2 + theta_c1_m1) + sin(theta_c_b + theta_c1_m1);
    A_tmp(2,2) = cos(theta_c_b - theta_b1_b2 + theta_c1_m1) - cos(theta_c_b + theta_c1_m1);
    A_tmp(2,3) = y_b1_b2*cos(theta_c_b + theta_c1_m1) - x_b1_b2*sin(theta_c_b + theta_c1_m1);
    b_tmp(2) = y_m1_m2 + y_c1_m1*cos(theta_c1_m1) - x_c1_m1*sin(theta_c1_m1) - y_c2_m2*cos(theta_b1_b2 - theta_c1_m1) - x_c2_m2*sin(theta_b1_b2 - theta_c1_m1);
    
    A = [A; A_tmp];
    b = [b; b_tmp];
    
    theta_b1_b2 = vec_b1_b2(3);
    theta_b1_b2_loop = theta_c1_m1 + theta_m1_m2 - theta_c2_m2;
    theta_b1_b2 = Trans_to_Period(theta_b1_b2, -pi, pi);
    theta_b1_b2_loop = Trans_to_Period(theta_b1_b2_loop, -pi, pi);
    
    A_theta = [A_theta; theta_b1_b2];
    b_theta = [b_theta; theta_b1_b2_loop];
end

vec_param = pinv(A)*b;
x_b_c = vec_param(1);
y_b_c = vec_param(2);
T_b_c(1:2, 3) = [x_b_c; y_b_c];

k_theta = pinv(A_theta)*b_theta*k_theta;
k_dist = k_dist*vec_param(3);

[ odo_refine ] = Proc_RefineOdo( odo_raw, k_dist, k_theta);
end

