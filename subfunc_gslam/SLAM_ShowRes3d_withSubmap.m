function [ ] = SLAM_ShowRes_withSubmap(...
    Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx, GraphDecomposeInfo)
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
set(gca, 'zlim', [0 sz_odo*z_gap]);
num_Alpha = numel(Alpha);

% draw odometry edge
for i = 1:(sz_odo-1)
    x_w_b1 = Alpha(3*i-2);
    y_w_b1 = Alpha(3*i-1);
    z_w_b1 = i*z_gap;
    
    x_w_b2 = Alpha(3*i+1);
    y_w_b2 = Alpha(3*i+2);
    z_w_b2 = (i+1)*z_gap;
    
    p_wNew_b1 = T_w_wNew_inv*[x_w_b1;y_w_b1;1];
    p_wNew_b2 = T_w_wNew_inv*[x_w_b2;y_w_b2;1];
    
    Idx_submap = GraphDecomposeInfo.SubmapIdx(i);
    switch mod(Idx_submap,3)
        case 0
            color = 'r';
        case 1
            color = [0;0.6;0];
        case 2
            color = 'b';
        otherwise
            color = 'b';
    end
    plot3([p_wNew_b1(1);p_wNew_b2(1)],[p_wNew_b1(2);p_wNew_b2(2)],[z_w_b1;z_w_b2],'Color',color);
end

% draw observation edge
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
    
    Idx_submap = GraphDecomposeInfo.SubmapIdx(idx_b);
    switch mod(Idx_submap,3)
        case 0
            color = 'r';
        case 1
            color = [0;0.6;0];
        case 2
            color = 'b';
        otherwise
            color = 'b';
    end    
    plot3([p_wNew_m(1);p_wNew_b(1)],[p_wNew_m(2);p_wNew_b(2)],[z_w_m;z_w_b],'Color',color);
end

% draw nodes
for i = 1 : num_Alpha/3
    x_w_m = Alpha(3*i-2);
    y_w_m = Alpha(3*i-1);
    z_w_m = z_gap*i;
    if z_w_m > sz_odo*z_gap
        z_w_m = 0;
    end
    
    p_wNew_m = T_w_wNew_inv*[x_w_m;y_w_m;1];
    SubmapIdx = GraphDecomposeInfo.SubmapIdx(i);
    if SubmapIdx == 0
        plot3(p_wNew_m(1),p_wNew_m(2),z_w_m,'.','Color','k', 'MarkerSize', 20);
    else
        switch mod(SubmapIdx, 3)
            case 0
                plot3(p_wNew_m(1),p_wNew_m(2),z_w_m,'.','Color','r', 'MarkerSize', 10);
            case 1
                plot3(p_wNew_m(1),p_wNew_m(2),z_w_m,'.','Color',[0;0.6;0], 'MarkerSize', 10);
            case 2
                plot3(p_wNew_m(1),p_wNew_m(2),z_w_m,'.','Color','b', 'MarkerSize', 10);
            otherwise
                plot3(p_wNew_m(1),p_wNew_m(2),z_w_m,'.','Color','r', 'MarkerSize', 10);
        end
    end
end

% draw mark
for i = 1:(num_Alpha-3*sz_odo)/3
    x_w_m = Alpha(3*sz_odo+3*i-2);
    y_w_m = Alpha(3*sz_odo+3*i-1);
    z_w_m = 0;
    p_wNew_m = T_w_wNew_inv*[x_w_m;y_w_m;1];
    plot3(p_wNew_m(1),p_wNew_m(2),z_w_m,'s','MarkerEdgeColor','k', 'MarkerFaceColor','k', 'MarkerSize', 10);
end

axis equal;
view(3);

end

