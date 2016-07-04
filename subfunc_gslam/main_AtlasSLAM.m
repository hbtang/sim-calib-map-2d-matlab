%% read data
clear;
load('data\HIT_G106\data_201507131623');
[ odo, mark ] = SLAM_ReadData( odo, mark);

%%%%%%%% debug %%%%%%%%
% mark.stamp = mark.stamp(1:10);
% mark.id = mark.id(1:10);
% mark.vec2d = mark.vec2d(1:10,:);
% mark.odoIdx = mark.odoIdx(1:10);
%%%%%%%% debug %%%%%%%%

[sz_odo, tmp] = size(odo.stamp);
[sz_mark, tmp] = size(mark.stamp);

%% Init
[ Alpha, Omega, Xi, hash_Alpha_odoIdx, hash_Alpha_markIdx ] = SLAM_Init( odo, mark, T_b_c );
Alpha_0 = Alpha;

%% SLAM main loop
num_loop = 3;
for i = 1:num_loop
    t1 = clock();
    
    %% Create constraint pool
    [ Constraint_Pool, Constraint_Hash ] = Create_Constraint_Pool( ...
        Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx, odo, mark, sz_odo, sz_mark, T_b_c );
    
    %% Decompose into subgraphs
    [ GraphDecomposeInfo ] = Graph_Decompose_by_Mark( Constraint_Pool, Constraint_Hash,...
        Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx);
    
    %% Create all submaps in information form
    [ Omega_submap_Pool, Xi_submap_Pool ] = Create_Submap_Pool( ...
        GraphDecomposeInfo, Constraint_Pool, Constraint_Hash);
    
    %% Do SLAM in upper level
    % state vector of key nodes: Alpha_key, ordered by the id of the submaps
    [ Omega_key, Xi_key, Alpha_key ] = Cal_InfoMatVec_Key( GraphDecomposeInfo, Constraint_Pool, ...
        Constraint_Hash, Omega_submap_Pool, Xi_submap_Pool, Alpha_0 );   
    
    %% Do SLAM in bottom level
    [ Alpha ] = Cal_Alpha_withKey_Atlas( GraphDecomposeInfo, Omega_submap_Pool, Xi_submap_Pool, Alpha, Alpha_key );
    
    t2 = clock();
    etime(t2,t1)
end


%% Show Results
[ hash_mark_odo ] = Create_Hash_mark_odo( mark, hash_Alpha_markIdx, hash_Alpha_odoIdx );
% SLAM_ShowRes(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx);
% SLAM_ShowRes_withSubmap(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx, GraphDecomposeInfo);
SLAM_ShowRes3d_withSubmap(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx, GraphDecomposeInfo);




