function [vec_ground, norm_vec_ground] = cal_ground(mark, markNum, pt_corner_mark)

%% calculate norm vector of the ground plane
pt_corner_cam = [];
for i = 1:markNum
    tvecTemp = mark.tvec(i,:);
    R_temp = rodrigues(mark.rvec(i,:));
    T_temp = [R_temp tvecTemp.' ; 0 0 0 1];
    pt_corner_cam_temp = transpose(T_temp*(pt_corner_mark.'));
    pt_corner_cam = [pt_corner_cam;pt_corner_cam_temp];
end

pt_corner_cam_pinv = pinv(pt_corner_cam(:,1:3));
vec_ground = -pt_corner_cam_pinv*pt_corner_cam(:,4);
norm_vec_ground = [vec_ground;0];
vec_ground = [vec_ground;1];
% recheck the plane result
% recheck_temp = pt_corner_cam * vec_ground;