function [ ] = SLAM_ShowRes3d(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx)
%SLAM_SHOWRES Summary of this function goes here
%% Show result
figure;
set(gca, 'xlim', [-2000 16000]);
set(gca, 'ylim', [-1000 7000]);

set(gca,'XTick',-2400:600:16000);
set(gca,'YTick',-1200:600:7000);
set(gca,'DataAspectRatio',[1 1 1]);
set(gca,'fontsize', 10);
grid on;
hold on;

%% Transform to wNew frame
T_w_wNew_inv = eye(3);

idx_alpha_m0 = Hash_Key_to_AlphaIdx(0,hash_Alpha_markIdx);
idx_alpha_m3 = Hash_Key_to_AlphaIdx(3,hash_Alpha_markIdx);

x_m0 = Alpha(idx_alpha_m0*3-2);
y_m0 = Alpha(idx_alpha_m0*3-1);
x_m3 = Alpha(idx_alpha_m3*3-2);
y_m3 = Alpha(idx_alpha_m3*3-1);

theta_w_wNew = atan2(y_m3-y_m0, x_m3-x_m0);
R_w_wNew = Trans_theta_to_R(theta_w_wNew);
T_w_wNew = [R_w_wNew [x_m0; y_m0]; 0 0 1];
T_w_wNew_inv = inv(T_w_wNew);

%% Draw
z_gap = 3;

% draw odometry
xVec_b = [];
yVec_b = [];
zVec_b = [];
for i = 1:(sz_odo-1)
    x_w_b1 = Alpha(3*i-2);
    y_w_b1 = Alpha(3*i-1);
    x_w_b2 = Alpha(3*i+1);
    y_w_b2 = Alpha(3*i+2);
    p_wNew_b1 = T_w_wNew_inv*[x_w_b1;y_w_b1;1];
    p_wNew_b2 = T_w_wNew_inv*[x_w_b2;y_w_b2;1];
    plot3([p_wNew_b1(1);p_wNew_b2(1)],[p_wNew_b1(2);p_wNew_b2(2)],[i*z_gap;(i+1)*z_gap],'.-');  
end

% draw mark
num_Alpha = numel(Alpha);
map_wNew = [];
for i = 1:(num_Alpha-3*sz_odo)/3
    x_w_m = Alpha(3*sz_odo+3*i-2);
    y_w_m = Alpha(3*sz_odo+3*i-1);
    p_wNew_m = T_w_wNew_inv*[x_w_m;y_w_m;1];
    plot3(p_wNew_m(1),p_wNew_m(2), 0,'s','MarkerEdgeColor','k', 'MarkerFaceColor','k', 'MarkerSize', 10);
    map_wNew = [map_wNew; p_wNew_m(1) p_wNew_m(2)];
end
% draw observation
[sz_obs,tmp] = size(hash_mark_odo);
for i = 1:sz_obs
    idx_m = hash_mark_odo(i,1);
    idx_b = hash_mark_odo(i,2);
    idx_alpha_m = Hash_Key_to_AlphaIdx(idx_m,hash_Alpha_markIdx);
    idx_alpha_b = Hash_Key_to_AlphaIdx(idx_b,hash_Alpha_odoIdx);

    x_w_m = Alpha(idx_alpha_m*3-2);
    y_w_m = Alpha(idx_alpha_m*3-1);
    z_w_m = 0;
    x_w_b = Alpha(idx_alpha_b*3-2);
    y_w_b = Alpha(idx_alpha_b*3-1);
    z_w_b = idx_alpha_b*z_gap;
    
    p_wNew_m = T_w_wNew_inv*[x_w_m;y_w_m;1];
    p_wNew_b = T_w_wNew_inv*[x_w_b;y_w_b;1];

    plot3([p_wNew_m(1);p_wNew_b(1)],[p_wNew_m(2);p_wNew_b(2)],[z_w_m, z_w_b],'Color','r');
end
set(gca, 'zlim', [0 sz_odo*z_gap]);
axis equal;
view(3);

end

