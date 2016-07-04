function [ Omega_key, Xi_key, Alpha_key ] = Cal_InfoMatVec_Key( GraphDecomposeInfo, Constraint_Pool, Constraint_Hash, Omega_submap_Pool, Xi_submap_Pool, Alpha_0 )
%CAL_INFOMATVEC_KEY Summary of this function goes here
%   Detailed explanation goes here
num_submap = max(GraphDecomposeInfo.SubmapIdx);
Omega_key = zeros(num_submap*3,num_submap*3);
Xi_key = zeros(num_submap*3,1);
vec_AlphaIdx_key = find(GraphDecomposeInfo.IfKey);
vec_LocalIdx_key = GraphDecomposeInfo.LocalIdx(vec_AlphaIdx_key);
vec_SubmapIdx_key = GraphDecomposeInfo.SubmapIdx(vec_AlphaIdx_key);
for i = 1:(num_submap-1)
    for j = (i+1):num_submap
        ifConnect = If_Submap_Connect( GraphDecomposeInfo, Constraint_Hash, i, j );
        if ifConnect
            Omega_submap_1 = Omega_submap_Pool{i};
            Xi_submap_1 = Xi_submap_Pool{i};
            Idx_submap_1 = i;
            Omega_submap_2 = Omega_submap_Pool{j};
            Xi_submap_2 = Xi_submap_Pool{j};
            Idx_submap_2 = j;
            [ Omega_submap_12, Xi_submap_12 ] = Cal_InfoMatVec_JointSubmap( GraphDecomposeInfo, Constraint_Pool, Constraint_Hash, ...
                Omega_submap_1, Xi_submap_1, Idx_submap_1, Omega_submap_2, Xi_submap_2, Idx_submap_2);
            
            % get index of all the key nodes in the joint map graph
            LocalIdx_key_1 = vec_LocalIdx_key(find(vec_SubmapIdx_key == Idx_submap_1));
            LocalIdx_key_2 = vec_LocalIdx_key(find(vec_SubmapIdx_key == Idx_submap_2));
            LocalIdx_key_2 = LocalIdx_key_2 + numel(Xi_submap_1)/3;
            
            IdxVec_key = [LocalIdx_key_1;LocalIdx_key_2];
            
            % margninalize
            [ Omega_key_12, Xi_key_12 ] = Cal_InfoMatVec_Marginal( Omega_submap_12, Xi_submap_12, IdxVec_key);
            
            % add to key level graph
            [ Omega_key, Xi_key ] = Add_InfoMatVec( Omega_key, Xi_key, Omega_key_12, Xi_key_12, Idx_submap_1, Idx_submap_2 );
        end
    end
end

% set initial information of the first landmark
% to fix the whole map in world frame.
AlphaIdx_key1 = Find_AlphaIdx_Key(GraphDecomposeInfo,1);
Alpha_Init_key1 = Alpha_0(3*AlphaIdx_key1-2:3*AlphaIdx_key1);
Q_inv = inv(diag([1;1;0.001]));
Omega_init = Q_inv;
Xi_init = Q_inv*Alpha_Init_key1;
Omega_key(1:3,1:3) = Omega_key(1:3,1:3) + Omega_init;
Xi_key(1:3) = Xi_key(1:3) + Xi_init;

% solve the SLAM problem
Alpha_key = Omega_key\Xi_key;
end

