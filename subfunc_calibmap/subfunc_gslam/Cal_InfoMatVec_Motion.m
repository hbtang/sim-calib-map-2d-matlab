function [ Omega_out, Xi_out ] = Cal_InfoMatVec_Motion( pose_1_SLAM, pose_2_SLAM, pose_1_odo, pose_2_odo )
%CAL_INFOMATVEC_MOTION Summary of this function goes here
%   Detailed explanation goes here

d_pose_1_2_odo = pose_2_odo - pose_1_odo;

dx_odo = d_pose_1_2_odo(1);
dy_odo = d_pose_1_2_odo(2);
dtheta_odo = d_pose_1_2_odo(3);
dtheta_SLAM = pose_2_SLAM(3) - pose_1_SLAM(3);

% set dtheta_odo close to state input
dtheta_odo = Trans_to_Period(dtheta_odo, dtheta_SLAM+pi, dtheta_SLAM-pi);
d_pose_1_2_odo(3) = dtheta_odo;

%% R: the covariance matrix of the motion error
dl_odo = sqrt(dx_odo*dx_odo + dy_odo*dy_odo);
R_sqrt = diag([0.01*dl_odo+1; 0.01*dl_odo+1; 2e-3]);
R = R_sqrt*R_sqrt;

%% Intermediate matrix
R_1 = Trans_theta_to_R(pose_1_SLAM(3));
R_1_odo = Trans_theta_to_R(pose_1_odo(3));
A1 = blkdiag(R_1', 1);
A2 = blkdiag(R_1_odo', 1);

b = -A2*d_pose_1_2_odo;
%% Omega: information matrix
R_inv = inv(R);
Omega_22 = A1'*R_inv*A1;
Omega_11 = A1'*R_inv*A1;
Omega_12 = -A1'*R_inv*A1;
Omega_21 = -A1'*R_inv*A1;

%% Xi: information vector
Xi_1 = A1'*R_inv*b;
Xi_2 = -A1'*R_inv*b;

%% debug
% A1*(pose_2_SLAM-pose_1_SLAM)+b

%% output
Omega_out = [Omega_11 Omega_12; Omega_21 Omega_22];
Xi_out = [Xi_1; Xi_2];
Omega_out = 0.5*(Omega_out+Omega_out.');

end

