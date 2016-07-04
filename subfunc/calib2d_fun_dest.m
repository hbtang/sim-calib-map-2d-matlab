function [ r ] = calib2d_fun_dest( x, A, b )
%CALIB2D_FUN_DEST Summary of this function goes here
%   Detailed explanation goes here
r = norm(A*x-b);
end

