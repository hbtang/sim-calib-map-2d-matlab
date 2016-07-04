function [ rvec ] = rot_to_parallel( vec_org, vec_dest )
%ROT_TO_PARALLEL Summary of this function goes here
vec1 = vec_org(1:3);
vec2 = vec_dest(1:3);
norm1 = norm(vec1);
norm2 = norm(vec2);
vec_rot = cross(vec1,vec2);
norm_temp = norm(vec_rot);
vec_rot = vec_rot/norm_temp; % now unit vector
theta = acos((vec1.'*vec2)/(norm1*norm2));
rvec = theta.*vec_rot;
end

