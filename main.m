%% init
[mark, markNum, markSize, pt_corner_mark, odo, odoNum, dataSerialNum] = init();

%% calculate norm vector of the ground plane
[vec_ground, norm_vec_ground] = cal_ground(mark, markNum, pt_corner_mark);

%% modify 3D coordinates into 2D plane
[T_cam_proj, mark] = proj_2_ground(mark,markNum,vec_ground);

%% calibration in 2d plane
[T_base_proj] = calib_cam_extrinsic(mark,markNum,odo);
% initial guess for VisionNav robot system
T_base_proj(1,3) = 387.45;
T_base_proj(2,3) = 0;

%% mapping & calibation
% [map, mapMarkNum] = mapping(mark, markNum, odo, T_base_proj);

odo_refined = odo;
k_compass = 1;
k_dist = 1;
traj = [];
calib_rec = [];

odo_rec = [];
traj_rec = [];
map_rec = [];

idNum = 10;

for i = (1:10)
    [map, mapMarkNum] = mapping(mark, markNum, odo_refined, T_base_proj, idNum);  
    [odo_refined, T_base_proj, k_compass, k_dist ] ...
        = refineOdo_byMap( odo_refined, odoNum, mark, markNum, map, T_base_proj, k_compass, k_dist ); 
    calib_tmp.k_compass = k_compass;
    calib_tmp.k_dist = k_dist;
    calib_tmp.T_base_proj = T_base_proj;    
    draw_results(mark,markNum,odo_refined,odoNum,traj,map,mapMarkNum,T_base_proj);
    calib_rec = [calib_rec calib_tmp];
    odo_rec = [odo_rec; odo_refined];
    traj_rec = [traj_rec; traj];
    map_rec = [map_rec; map];
end

%% relocalization
[traj] = relocal(odo_refined, odoNum, mark, markNum, map, mapMarkNum, T_base_proj);

%% draw result
% draw_results(mark,markNum,odo,odoNum,traj,map,mapMarkNum,T_base_proj);
draw_results(mark,markNum,odo_refined,odoNum,traj,map,mapMarkNum,T_base_proj);

%% show all calibration and mapping
display_results;

%% tmp
% i = 1;
% draw_results(mark,markNum,odo_rec(i),odoNum,[],map_rec(1+10*(i-1):10*i),mapMarkNum,calib_rec(i).T_base_proj);
% 
% map_real = map;
% map_real(1).vec = [0;0;0];
% map_real(2).vec = [0;-4200;0];
% map_real(3).vec = [0;-8400;0];
% map_real(4).vec = [0;-14400;0];
% map_real(5).vec = [3600;-14400;0];
% map_real(6).vec = [6600;-8400;0];
% map_real(7).vec = [6600;-4200;0];
% map_real(8).vec = [6600;0;0];
% map_real(9).vec = [3600;0;0];
% map_real(10).vec = [3600;-6600;0];  
% e_vec = [];
% e = zeros(20,1);
% for i = (1:10)
%     map_i = map_rec(1+10*(i-1):10*i);    
%     for j = 1:10
%         e(2*j-1) = map_i(j).vec(1) - map_real(j).vec(1);
%         e(2*j) = map_i(j).vec(2) - map_real(j).vec(2);
%     end
%     e_vec = [e_vec e];    
% end
% plot(e_vec.','LineWidth',2);
% 
% r_vec = [];
% for i = 1:10
%     r = [calib_rec(i).k_compass;calib_rec(i).k_dist];
%     r_vec = [r_vec r];
% end
% plot(r_vec.','LineWidth',2);
% 
% r_vec = [];
% for i = 1:10
%     r = [calib_rec(i).T_base_proj(1,3);calib_rec(i).T_base_proj(2,3)];
%     r_vec = [r_vec r];
% end
% plot(r_vec.','LineWidth',2);
% 
% draw_results(mark,markNum,odo_rec(i),0,[],map_rec(1+10*(i-1):10*i),mapMarkNum,calib_rec(i).T_base_proj);