function [ Alpha ] = Cal_Alpha_withKey_Atlas( GraphDecomposeInfo, Omega_submap_Pool, Xi_submap_Pool, Alpha, Alpha_key )
%CAL_ALPHA_WITHKEY_ATLAS Summary of this function goes here

num_submap = max(GraphDecomposeInfo.SubmapIdx);
% cell with state vector of each vector
Alpha_submap_Pool = cell(num_submap, 1);
for i = 1:num_submap
    [ AlphaIdx_key, LocalIdx_key ] = Find_AlphaIdx_Key( GraphDecomposeInfo, i);
    Alpha_k_now = Alpha_key(3*i-2:3*i);
    
    IdxVec_Alpha_now = find(GraphDecomposeInfo.SubmapIdx == i);
    IdxVec_Alpha_now = Trans_IdxVecNode_IdxVecAlpha(IdxVec_Alpha_now);
    Alpha_0_now = Alpha(IdxVec_Alpha_now);
    
    
    [ Omega_submap_n, Xi_submap_n ] = Cal_InfoMatVec_Conditional( ...
        Omega_submap_Pool{i}, Xi_submap_Pool{i}, Alpha_k_now, LocalIdx_key);
    Alpha_n_now = Omega_submap_n\Xi_submap_n;
    
    IdxVec_Local_k = (3*LocalIdx_key-2 : 3*LocalIdx_key)';
    IdxVec_Local_all = (1:numel(Xi_submap_Pool{i}))';
    IdxVec_Local_n = IdxVec_Local_all;
    IdxVec_Local_n(IdxVec_Local_k) = []; 
    
    Alpha_now = zeros(numel(Xi_submap_Pool{i}),1);
    Alpha_now(IdxVec_Local_k) = Alpha_k_now;
    Alpha_now(IdxVec_Local_n) = Alpha_n_now;
    Alpha_submap_Pool{i} = Alpha_now;
    
    % generate Alpha for whole graph
    IdxVec_Alpha_now = find(GraphDecomposeInfo.SubmapIdx == i);
    IdxVec_Alpha_now = Trans_IdxVecNode_IdxVecAlpha(IdxVec_Alpha_now);
    Alpha(IdxVec_Alpha_now) = Alpha_now;
end

end

