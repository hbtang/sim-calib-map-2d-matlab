function [ R ] = cal_2d_trans_matrix( x, y, theta )
%CAL_ROT_MATRIX Summary of this function goes here
%   Detailed explanation goes here
R = [cos(theta) -sin(theta) x;
    sin(theta) cos(theta) y;
    0 0 1];
end

