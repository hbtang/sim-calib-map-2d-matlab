function [ Aij, Bij, eij ] = cal_AB_opt( vec_i, vec_j, constr_T )
%CAL_AB_OPT Summary of this function goes here
%   Detailed explanation goes here

theta_i = vec_i(3);
theta_j = vec_j(3);
theta_ij = atan2(constr_T(2,1),constr_T(1,1));
x_i = vec_i(1);
y_i = vec_i(2);
x_j = vec_j(1);
y_j = vec_j(2);
Rij = constr_T(1:2,1:2);
Ri = [cos(theta_i) -sin(theta_i); sin(theta_i) cos(theta_i)];
tij = constr_T(1:2,3);
ti = vec_i(1:2);
tj = vec_j(1:2);
d_Ri_thetai = [-sin(theta_i) -cos(theta_i); cos(theta_i) -sin(theta_i)];
Aij = [-Rij'*Ri' Rij'*d_Ri_thetai'*(tj - ti);
    0 0 -1];
Bij = [Rij'*Ri' [0;0];
    0 0 1];
eij = [Rij'*(Ri'*(tj-ti)-tij);theta_j-theta_i-theta_ij];
end

