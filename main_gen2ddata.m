%% init
[mark, markNum, markSize, pt_corner_mark, odo, odoNum, dataSerialNum] = init();

%% calculate norm vector of the ground plane
[vec_ground, norm_vec_ground] = cal_ground(mark, markNum, pt_corner_mark);

%% modify 3D coordinates into 2D plane
[T_cam3d_cam2d, mark] = proj_2_ground(mark,markNum,vec_ground);

%% calibration in 2d plane
[T_b_c] = calib_cam_extrinsic(mark,markNum,odo);
% initial guess for VisionNav robot system
T_b_c(1,3) = 200;
T_b_c(2,3) = 0;

save('data.mat', 'odo', 'mark', 'T_b_c', 'T_cam3d_cam2d');