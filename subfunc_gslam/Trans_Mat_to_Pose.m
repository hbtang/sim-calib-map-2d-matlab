function [ pose ] = Trans_Mat_to_Pose( T )
%TRANS_MAT_TO_POSE Summary of this function goes here
%   Detailed explanation goes here
pose = zeros(3,1);
pose(1) = T(1,3);
pose(2) = T(2,3);
pose(3) = atan2(T(2,1),T(1,1));
end

