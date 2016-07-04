function [c,ceq] = calib2d_fun_constr( x )
%CALIB2D_FUN_CONSTR Summary of this function goes here
%   Detailed explanation goes here

c = [];
ceq = norm(x(3:4))-1;

end

