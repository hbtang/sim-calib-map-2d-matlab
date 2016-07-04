function [T_base_proj] = calib_cam_extrinsic(mark,markNum,odo)

%% calibration in 2d plane
% calibrate the 2d transformation matrix from robot frame to camera proj
% frame

A = []; B = [];
for i = 1:(markNum-1)
    if (mark.id(i) == mark.id(i+1)) && (mark.stamp(i)-mark.stamp(i+1) < 10)
        stamp_temp = mark.stamp(i);
        theta_w_b1 = odo.theta(stamp_temp);
        theta_w_b2 = odo.theta(stamp_temp+1);
        T_w_b1 = [cos(theta_w_b1) -sin(theta_w_b1) odo.x(stamp_temp);
            sin(theta_w_b1) cos(theta_w_b1) odo.y(stamp_temp);
            0 0 1;
            ];
        T_w_b2 = [cos(theta_w_b2) -sin(theta_w_b2) odo.x(stamp_temp+1);
            sin(theta_w_b2) cos(theta_w_b2) odo.y(stamp_temp+1);
            0 0 1;
            ];
        T_c1_m = [cos(mark.vec2d(i,3)) -sin(mark.vec2d(i,3)) mark.vec2d(i,1);
            sin(mark.vec2d(i,3)) cos(mark.vec2d(i,3)) mark.vec2d(i,2);
            0 0 1;
            ];
        T_c2_m = [cos(mark.vec2d(i+1,3)) -sin(mark.vec2d(i+1,3)) mark.vec2d(i+1,1);
            sin(mark.vec2d(i+1,3)) cos(mark.vec2d(i+1,3)) mark.vec2d(i+1,2);
            0 0 1;
            ];
        T_b = inv(T_w_b1)*T_w_b2;
        T_c = T_c1_m*inv(T_c2_m);
        A_temp = [
            T_b(1,1)-1 T_b(1,2) -T_c(1,3) T_c(2,3);
            T_b(2,1) T_b(2,2)-1 -T_c(2,3) -T_c(1,3);
            ];
        B_temp = [-T_b(1,3);-T_b(2,3)];
        A = [A;A_temp];
        B = [B;B_temp];
    end
end
param_2d_calib = fmincon(@(x)calib2d_fun_dest(x, A, B),[0;0;1;0],[],[],[],[],[],[],@calib2d_fun_constr);
% param_2d_calib = pinv(A)*B;
theta_base_camproj = atan2(param_2d_calib(4),param_2d_calib(3));
x_base_proj = param_2d_calib(1);
y_base_proj = param_2d_calib(2);
T_base_proj = [
    cos(theta_base_camproj) -sin(theta_base_camproj) x_base_proj;
    sin(theta_base_camproj) cos(theta_base_camproj) y_base_proj;
    0 0 1;
    ];