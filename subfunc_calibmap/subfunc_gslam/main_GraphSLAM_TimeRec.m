%% pick fist num_odo_max measurements
load('data\HIT_G106\data_201507131623');
[ odo_all, mark_all ] = SLAM_ReadData(odo, mark);
cmp_intv_rec = [];
for num_odo_max = 100:100:numel(odo_all.stamp)
    
odo.stamp = odo_all.stamp(1:num_odo_max);
odo.x = odo_all.x(1:num_odo_max);
odo.y = odo_all.y(1:num_odo_max);
odo.theta = odo_all.theta(1:num_odo_max);
stamp_max = odo.stamp(end);
vec_mark = find(mark_all.stamp <= stamp_max);
mark.stamp = mark_all.stamp(vec_mark);
mark.id = mark_all.id(vec_mark);
mark.vec2d = mark_all.vec2d(vec_mark,:);
mark.odoIdx = mark_all.odoIdx(vec_mark);

%% set size
[sz_odo, tmp] = size(odo.stamp);
[sz_mark, tmp] = size(mark.stamp);

%% Init
[ Alpha, Omega, Xi, hash_Alpha_odoIdx, hash_Alpha_markIdx ] = SLAM_Init( odo, mark, T_b_c );
Alpha_0 = Alpha;

num_loop = 1;
for i = 1:num_loop 
    %% start clock
    t1 = clock();
    
    %% Reset Optimization Problem
    [ Omega, Xi, hash_mark_odo ] = SLAM_ResetOptFun( Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx,...
        odo, mark, T_b_c, sz_odo, sz_mark );
    %% Solve graph SLAM
    Alpha = Omega\Xi;
    
    
    %% Test: marginalize all other nodes
%     load('Idx_Bnd.mat');    
%     [Omega_key, Xi_key] = Cal_InfoMatVec_Marginal(Omega,Xi,IdxVec_Alpha_Bnd);
%     Alpha_key = Omega_key\Xi_key;
%     IdxVec_tmp = Trans_IdxVecNode_IdxVecAlpha(IdxVec_Alpha_Bnd);    
%     Alpha(IdxVec_tmp) - Alpha_key
    
    %% end clock
    t2 = clock();
    comp_intv = etime(t2,t1); 
end

num_odo_max
comp_intv
cmp_intv_rec = [cmp_intv_rec; num_odo_max comp_intv];
end
plot(cmp_intv_rec(:,1), cmp_intv_rec(:,2));

%% Show Results
% SLAM_ShowRes(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx);
% SLAM_ShowRes3d(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx);



