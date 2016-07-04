function [ pose_i_j ] = Cal_Pose_i_j( pose_i, pose_j )
%CAL_POSE_I_J Summary of this function goes here
%   Detailed explanation goes here
theta_i = pose_i(3);
pose_i_j = [cos(-theta_i) -sin(-theta_i) 0;...
    sin(-theta_i) cos(-theta_i) 0;
    0 0 1]*(pose_j - pose_i);
end

