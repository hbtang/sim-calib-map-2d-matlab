function [ vec_pt_proj ] = cal_projective_pt( vec_pt, vec_plane )
%CAL_PROJECTIVE_PT Summary of this function goes here
%   return projective point to a plane
dist_gen = dot(vec_pt, vec_plane);
vec_norm_plan = [vec_plane(1:3);0];
n2 = dot(vec_norm_plan,vec_norm_plan);
gamma = -dist_gen/n2;
vec_pt_proj = vec_pt + gamma.*vec_norm_plan;
end

