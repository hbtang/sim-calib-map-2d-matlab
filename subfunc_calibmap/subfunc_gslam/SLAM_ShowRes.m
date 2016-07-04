function [ map_wNew ] = SLAM_ShowRes(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx)
%SLAM_SHOWRES Summary of this function goes here
%% Show result
% figure;
hold off;
plot(0,0);
grid on;
hold on;
% set(gca, 'xlim', [-5000 19000]);
% set(gca, 'ylim', [-4000 10000]);
set(gca, 'XTick',-100000:5000:100000);
set(gca, 'YTick',-100000:5000:100000);
set(gca, 'DataAspectRatio',[1 1 1]);
set(gca, 'fontsize', 10);
set(gcf, 'Position', [1,1,1280,720]);


%% Transform to wNew frame
T_w_wNew_inv = eye(3);

idx_alpha_m0 = Hash_Key_to_AlphaIdx(0,hash_Alpha_markIdx);
idx_alpha_m3 = Hash_Key_to_AlphaIdx(3,hash_Alpha_markIdx);

x_m0 = Alpha(idx_alpha_m0*3-2);
y_m0 = Alpha(idx_alpha_m0*3-1);
x_m3 = Alpha(idx_alpha_m3*3-2);
y_m3 = Alpha(idx_alpha_m3*3-1);

theta_w_wNew = atan2(y_m3-y_m0, x_m3-x_m0) - pi/2;
R_w_wNew = Trans_theta_to_R(theta_w_wNew);
T_w_wNew = [R_w_wNew [x_m0; y_m0]; 0 0 1];
T_w_wNew_inv = inv(T_w_wNew);

%% Draw
% draw odometry
for i = 1:(sz_odo-1)
    x_w_b1 = Alpha(3*i-2);
    y_w_b1 = Alpha(3*i-1);
    x_w_b2 = Alpha(3*i+1);
    y_w_b2 = Alpha(3*i+2);    
    p_wNew_b1 = T_w_wNew_inv*[x_w_b1;y_w_b1;1];
    p_wNew_b2 = T_w_wNew_inv*[x_w_b2;y_w_b2;1];
    plot([p_wNew_b1(1);p_wNew_b2(1)],[p_wNew_b1(2);p_wNew_b2(2)],'.-');    
end

% draw mark
num_Alpha = numel(Alpha);
map_wNew = [];
for i = 1:(num_Alpha-3*sz_odo)/3
    x_w_m = Alpha(3*sz_odo+3*i-2);
    y_w_m = Alpha(3*sz_odo+3*i-1);
    theta_w_m = Alpha(3*sz_odo+3*i);
    
    theta_wNew_m = -theta_w_wNew + theta_w_m;
    p_wNew_m = T_w_wNew_inv*[x_w_m;y_w_m;1];    
    plot(p_wNew_m(1),p_wNew_m(2),'s','MarkerEdgeColor','k', 'MarkerFaceColor','k', 'MarkerSize', 10);;   
    
    map_wNew = [map_wNew; p_wNew_m(1) p_wNew_m(2) theta_wNew_m];
end
% vpa(map_wNew,6)

% draw observation
[sz_obs,tmp] = size(hash_mark_odo);
for i = 1:sz_obs
    idx_m = hash_mark_odo(i,1);
    idx_b = hash_mark_odo(i,2);
    idx_alpha_m = Hash_Key_to_AlphaIdx(idx_m,hash_Alpha_markIdx);
    idx_alpha_b = Hash_Key_to_AlphaIdx(idx_b,hash_Alpha_odoIdx);
    
    x_w_m = Alpha(idx_alpha_m*3-2);
    y_w_m = Alpha(idx_alpha_m*3-1);
    x_w_b = Alpha(idx_alpha_b*3-2);
    y_w_b = Alpha(idx_alpha_b*3-1);
    
    p_wNew_m = T_w_wNew_inv*[x_w_m;y_w_m;1];   
    p_wNew_b = T_w_wNew_inv*[x_w_b;y_w_b;1];
    
    plot([p_wNew_m(1);p_wNew_b(1)],[p_wNew_m(2);p_wNew_b(2)],'Color','r');
end



end

