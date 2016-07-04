function [ R ] = Trans_theta_to_R( theta )
%TRANS_THETA_TO_R Summary of this function goes here
%   Detailed explanation goes here
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end

