function [ T ] = Trans_Pose_to_Mat( pose )
%TRANS_POSE_TO_MAT Summary of this function goes here
%   Detailed explanation goes here
theta = pose(3);
x = pose(1);
y = pose(2);
T = [cos(theta) -sin(theta) x;...
    sin(theta) cos(theta) y;...
    0 0 1];
end

