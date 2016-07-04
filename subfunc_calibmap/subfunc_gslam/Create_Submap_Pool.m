function [ Omega_submap_Pool, Xi_submap_Pool ] = Create_Submap_Pool( GraphDecomposeInfo, Constraint_Pool, Constraint_Hash)
%CREATE_SUBMAP_POOL Summary of this function goes here
%   Detailed explanation goes here
% Create Submap in information form
num_submap = max(GraphDecomposeInfo.SubmapIdx);
Omega_submap_Pool = cell(num_submap,1);
Xi_submap_Pool = cell(num_submap,1);
for i = 1:num_submap
    [ Omega_submap_tmp, Xi_submap_tmp ] = Cal_InfoMatVec_Submap( GraphDecomposeInfo, ...
        Constraint_Pool, Constraint_Hash, i);
    Omega_submap_Pool{i} = Omega_submap_tmp;
    Xi_submap_Pool{i} = Xi_submap_tmp;
end

end

