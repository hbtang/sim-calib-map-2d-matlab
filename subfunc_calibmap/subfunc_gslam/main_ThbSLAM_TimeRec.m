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

[sz_odo, tmp] = size(odo.stamp);
[sz_mark, tmp] = size(mark.stamp);

%% Init
[ Alpha, Omega, Xi, hash_Alpha_odoIdx, hash_Alpha_markIdx ] = SLAM_Init( odo, mark, T_b_c );
Alpha_0 = Alpha;

%% SLAM main loop

num_loop = 1;
for i = 1:num_loop
    t1 = clock();
    
    %% Create constraint pool
    [ Constraint_Pool, Constraint_Hash ] = Create_Constraint_Pool( ...
        Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx, odo, mark, sz_odo, sz_mark, T_b_c );
    
%     %% Test part 1: create Omega and Xi for all
%     [sz_tmp,tmp] = size(Constraint_Hash);
%     Omega_all = zeros(sz_tmp*3,sz_tmp*3);
%     Xi_all = zeros(sz_tmp*3,1);    
%     [rows, cols] = find(Constraint_Hash);
%     num_cnstr = numel(rows);    
%     for i = 1:num_cnstr
%         Omega_add = Constraint_Pool(Constraint_Hash(rows(i),cols(i))).Omega;
%         Xi_add = Constraint_Pool(Constraint_Hash(rows(i),cols(i))).Xi;
%         Idx_1_add = Constraint_Pool(Constraint_Hash(rows(i),cols(i))).AlphaIdx_1;
%         Idx_2_add = Constraint_Pool(Constraint_Hash(rows(i),cols(i))).AlphaIdx_2;
%         [ Omega_all, Xi_all ] = Add_InfoMatVec( Omega_all, Xi_all, ...
%             Omega_add, Xi_add, Idx_1_add, Idx_2_add );
%     end
    
    
    %% Decompose into subgraphs
    [ GraphDecomposeInfo ] = Graph_Dcmps_by_Mark_with_Bndry( Constraint_Pool, Constraint_Hash,...
        Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx);
    
%     %% Test part 2: decompose Omega_all into submaps    
%     IdxVec_0 = find(GraphDecomposeInfo.SubmapIdx == 0);
%     num_submap = max(GraphDecomposeInfo.SubmapIdx);
%     IdxVec_pool = cell(num_submap,1);
%     for i = 1:num_submap
%         IdxVec_pool{i} = find(GraphDecomposeInfo.SubmapIdx == i);
%     end 
%     T = zeros(num_submap,num_submap);
%     for i = 1 : num_submap
%         for j = 1 : num_submap            
%             IdxVec_a1 = Trans_IdxVecNode_IdxVecAlpha(IdxVec_pool{i});
%             IdxVec_a2 = Trans_IdxVecNode_IdxVecAlpha(IdxVec_pool{j});
%             tmp = numel(find(Omega_all(IdxVec_a1,IdxVec_a2)));
%             if tmp ~= 0 && i ~= j
%                 db = 1;
%             end
%             
%             T(i,j) = tmp;
%         end
%     end    
%     spy(T);
    
    
    
    %% Do SLAM in upper level
    [ Submap_with_Bndry_Pool, Alpha_Bnd ] = ...
    SLAM_Thb_HLv( GraphDecomposeInfo, Constraint_Pool, Constraint_Hash);
    
    %% Do SLAM in bottom level
    [ Alpha ] = SLAM_Thb_LLv( GraphDecomposeInfo, Submap_with_Bndry_Pool, Alpha, Alpha_Bnd);  
   
    t2 = clock();
    comp_intv = etime(t2,t1);
end

num_odo_max
comp_intv
cmp_intv_rec = [cmp_intv_rec; num_odo_max comp_intv];
end
plot(cmp_intv_rec(:,1), cmp_intv_rec(:,2));

%% Show Results
% [ hash_mark_odo ] = Create_Hash_mark_odo( mark, hash_Alpha_markIdx, hash_Alpha_odoIdx );
% SLAM_ShowRes(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx);
% SLAM_ShowRes_withSubmap(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx, GraphDecomposeInfo);
% SLAM_ShowRes3d_withSubmap(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx, GraphDecomposeInfo);




