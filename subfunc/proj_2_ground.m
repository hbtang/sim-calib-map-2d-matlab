function [T_cam_proj, mark_output] = proj_2_ground(mark,markNum,vec_ground)

%% modify 3D coordinates into 2D plane
% for camera frame, the projective center to the ground is the center in 2D
% and the x axis in 3D space project into 2D plane and become -y axis;

% for landmark, y axis would be modified to be parallel with ground norm by
% pure rotation, then the z axis would becom new x axis and x axis would
% become new y axis

% refine landmarks to align y axis to norm vector of ground
mark.rvec_refined = [];
norm_vec_ground = [vec_ground(1:3);0];
for i = 1:markNum
    tvecTemp = mark.tvec(i,:);
    R_temp = rodrigues(mark.rvec(i,:));
    T_temp = [R_temp tvecTemp.' ; 0 0 0 1];
    invT_temp = inv(T_temp);
    vec_y_refined_temp = invT_temp*norm_vec_ground;
    vec_y_temp = [0;1;0;0];
    rvec2_temp = rot_to_parallel(vec_y_temp,vec_y_refined_temp);
    R2_temp = rodrigues(rvec2_temp);
    R_refine_temp = R_temp*R2_temp;
    rvec_refine_temp = rodrigues(R_refine_temp);
    mark.rvec_refined = [mark.rvec_refined ; rvec_refine_temp.'];
    %%%% test result %%%%
    %     R_refine_temp*vec_y_temp(1:3)
    %%%% test result %%%%
end

% define the origin frame in 2D plane
% origin point
pt_origin_cam_proj = cal_projective_pt([0;0;0;1],vec_ground);
tvec_cam_proj = pt_origin_cam_proj(1:3);
axis_y_cam_proj = -cal_projective_pt([1;0;0;0],vec_ground);
axis_z_cam_proj = norm_vec_ground./norm(norm_vec_ground);
axis_x_cam_proj = cross(axis_y_cam_proj(1:3),axis_z_cam_proj(1:3));
axis_x_cam_proj = [axis_x_cam_proj;0];
R_cam_proj = [axis_x_cam_proj(1:3) axis_y_cam_proj(1:3) axis_z_cam_proj(1:3)];
rvec_cam_proj = rodrigues(R_cam_proj);
% get transformation matrix from camera to its projection in 2D frame
T_cam_proj = [R_cam_proj tvec_cam_proj; 0 0 0 1];
invT_cam_proj = inv(T_cam_proj);

% project landmark into 2D frame
mark.vec2d = [];
for i = 1:markNum
    rvec_refine_temp = mark.rvec_refined(i,:);
    R_refine_temp = rodrigues(rvec_refine_temp);
    tvec_temp = mark.tvec(i,:).';
    T_temp = [R_refine_temp tvec_temp; 0 0 0 1];
    T_temp_proj_mark = invT_cam_proj*T_temp;
    R_temp_proj_mark = T_temp_proj_mark(1:3,1:3);
    x_temp_proj = T_temp_proj_mark(1,4);
    y_temp_proj = T_temp_proj_mark(2,4);
    % calculate theta of landmark in 2D
    R_temp_proj_mark2 = R_temp_proj_mark*[0 1 0; 0 0 1; 1 0 0];
    rvec_refine_temp_2 = rodrigues(R_temp_proj_mark2);
    theta_temp_proj = rvec_refine_temp_2(3);
    mark.vec2d = [mark.vec2d; x_temp_proj y_temp_proj theta_temp_proj];
end

mark_output = mark;