function [ Omega_out, Xi_out ] = Cal_InfoMatVec_Observation( pose_b, pose_m, pose_c_m, T_b_c )
%CAL_INFOMATVEC_MOTION Summary of this function goes here

pose_b_c = Trans_Mat_to_Pose(T_b_c);

%% Q: observation error covariance matrix
Q_sqrt = diag([5; 5; 3*pi/180]);
Q = Q_sqrt*Q_sqrt;
Q_inv = inv(Q);

%% A, b, temp matrix and vector

theta_b = pose_b(3);
theta_b_c = pose_b_c(3);
theta_c = theta_b+theta_b_c;
theta_m = pose_m(3);

% transform theta_c_m periodically close to status
theta_c_m = pose_c_m(3);
theta_c_m = Trans_to_Period(theta_c_m, theta_m-theta_c-pi, theta_m-theta_c+pi);
pose_c_m(3) = theta_c_m;

R_c = Trans_theta_to_R(theta_c);
A = blkdiag(R_c',1);

R_b_c = Trans_theta_to_R(theta_b_c);
b = - blkdiag(R_b_c',1)*pose_b_c - pose_c_m;

%% generate Omega and Xi
Omega_bb = A'*Q_inv*A;
Omega_mm = A'*Q_inv*A;
Omega_mb = -A'*Q_inv*A;
Omega_bm = -A'*Q_inv*A;
Xi_b = A'*Q_inv*b;
Xi_m = -A'*Q_inv*b;

Omega_out = [Omega_bb Omega_bm; Omega_mb Omega_mm];
Xi_out = [Xi_b; Xi_m];
alpha_now = [pose_b;pose_m];

% A*(pose_m-pose_b) + b
Omega_out = 0.5*(Omega_out+Omega_out.');

end

