%% read data
clear;

load('data_201512211433');
[ odo_raw, mark ] = SLAM_ReadData(odo, mark);

sz_odo= numel(odo_raw.stamp);
sz_mark = numel(mark.stamp);

%% video init
% VideoObj = VideoWriter('result/CalibAndMapping.avi', 'Motion JPEG AVI' );
% VideoObj.FrameRate = 5;
% open(VideoObj);

%% plot init
fig = figure;
grid on;
hold on;
% set(gca, 'xlim', [-5000 19000]);
% set(gca, 'ylim', [-4000 10000]);
% set(gca, 'XTick',-2400:600:16000);
% set(gca, 'YTick',-1200:600:7000);
set(gca, 'DataAspectRatio',[1 1 1]);
set(gca, 'fontsize', 10);
set(gcf, 'Position', [1,1,1280,720]);

%% distort odometry
k_theta = 1;
k_dist = 1;
T_b_c(1:2,3) = [200;0];
[ odo_refine ] = Proc_RefineOdo( odo_raw, k_dist, k_theta );
rec_param = [k_dist, k_theta, T_b_c(1,3), T_b_c(2,3)];

%% debug for CZ
% T_b_c = [ 0.018731, -0.999825, 297.805192;... 
%         0.999825, 0.018731, -1.073778;... 
%         0.000000, 0.000000, 1.000000
%         ];

%% slam init
[ Alpha, Omega, Xi, hash_Alpha_odoIdx, hash_Alpha_markIdx ] = SLAM_Init( odo_refine, mark, T_b_c );
rec_map = cell(0,0);
num_loop = 10;

for i = 1:num_loop
    %% start clock
    t1 = clock();
    
    %% reset optimization problem
    [ Omega, Xi, hash_mark_odo ] = SLAM_ResetOptFun( Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx,...
        odo_refine, mark, T_b_c, sz_odo, sz_mark );
    
    %% solve graph SLAM
    Alpha = Omega\Xi;
    
    %% refine parameters: camera extrinsics and odometry param
    [ T_b_c, k_dist, k_theta, Alpha, odo_refine] = Proc_RefineParam( Alpha, odo_raw, k_dist, k_theta, sz_odo, mark, sz_mark, T_b_c, ...
        hash_Alpha_odoIdx, hash_Alpha_markIdx );
    
    rec_param_tmp = [k_dist, k_theta, T_b_c(1,3), T_b_c(2,3)];
    rec_param = [rec_param; rec_param_tmp];
        
    %% draw results
    map_tmp = SLAM_ShowRes(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx);
    rec_map{i,1} = map_tmp;
    pause(0.1);
%     frame = getframe(fig);
%     writeVideo(VideoObj, frame);
%     saveas(fig, ['result/lp', num2str(i), '.emf']);
    
    %% end clock
    t2 = clock();
    disp(['Loop ', num2str(i), ', using ',num2str(etime(t2,t1)), ' sec.',...
        'k_dist:', num2str(k_dist), ', k_theta:', num2str(k_theta)])
end

% close(VideoObj);

%% write into file
outputFilePath = ['thb_calib_map.yml'];
outputFileId = fopen(outputFilePath,'w');
fprintf(outputFileId,'%%YAML:1.0\n');
writeYmlMatrix( outputFileId, T_cam3d_cam2d, 'T_cam3d_cam2d', 'f');
writeYmlMatrix( outputFileId, T_b_c, 'T_base_cam', 'f');
writeYmlMatrix( outputFileId, hash_Alpha_markIdx(:,2), 'vec_id', 'd');
writeYmlMatrix( outputFileId, map_tmp, 'map_matrix', 'f');
fclose(outputFileId);

%% show results
% rec_map_new = [];
% map_new_ref = [...
%     0,0,2400,0,4200,0,5400,0,6600,0,8400,0,9600,0,10800,0,12000,0,...
%     14400,0,14400,2400,14400,3600,14400,4800,14400,6600,12600,6600,...
%     11400,6600,10200,6600,8400,6600,7200,6600,5400,6600,4200,6600,3000,6600,1800,6600,...
%     0,3600,0,2400,0,1200,1200,0,4200,-600,5400,-600,6600,-600,8400,-600,9600,-600,10800,-600,13200,0,14400,1200,...
%     5400,4800,5400,3600,5400,2400,5400,1200];
% for i = 1:numel(rec_map)
%     map_new_tmp = [];
%     map_tmp = rec_map{i};
%     for j = 1:39        
%         map_new_tmp = [map_new_tmp, map_tmp(j,:)];
%     end
%     rec_map_new = [rec_map_new; map_new_tmp-map_new_ref];    
% end
% plot(rec_map_new)

