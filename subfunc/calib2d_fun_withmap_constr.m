function [ c,ceq ] = calib2d_fun_withmap_constr( x, y )
%CALIB2D_FUN_CONSTR Summary of this function goes here
%   Detailed explanation goes here

c = [];
ceq = norm(x(3)-y);

end
