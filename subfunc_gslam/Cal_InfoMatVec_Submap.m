function [ Omega, Xi ] = Cal_InfoMatVec_Submap( GraphDecomposeInfo, Constraint_Pool, Constraint_Hash, Idx_submap )
%CAL_INFOMATVEC_KEY Summary of this function goes here
%   Detailed explanation goes here

Indice_Alpha = find(GraphDecomposeInfo.SubmapIdx == Idx_submap);
Constraint_Hash_submap = Constraint_Hash(Indice_Alpha, Indice_Alpha);

[row, col] = find(Constraint_Hash_submap ~= 0);

num_constraint = numel(row);
num_node = numel(Indice_Alpha);

Omega = zeros(3*num_node, 3*num_node);
Xi = zeros(3*num_node, 1);

for i = 1:num_constraint
    Idx_ConstraintPool_now = Constraint_Hash_submap(row(i), col(i));
    Omega_now = Constraint_Pool(Idx_ConstraintPool_now).Omega;
    Xi_now = Constraint_Pool(Idx_ConstraintPool_now).Xi;
    
    Indice_Alpha_submap_1 = 3*row(i)-2 : 3*row(i);
    Indice_Alpha_submap_2 = 3*col(i)-2 : 3*col(i);
    
    Omega(Indice_Alpha_submap_1,Indice_Alpha_submap_1) = ...
        Omega(Indice_Alpha_submap_1,Indice_Alpha_submap_1) + Omega_now(1:3,1:3);
    Omega(Indice_Alpha_submap_1,Indice_Alpha_submap_2) = ...
        Omega(Indice_Alpha_submap_1,Indice_Alpha_submap_2) + Omega_now(1:3,4:6);
    Omega(Indice_Alpha_submap_2,Indice_Alpha_submap_1) = ...
        Omega(Indice_Alpha_submap_2,Indice_Alpha_submap_1) + Omega_now(4:6,1:3);
    Omega(Indice_Alpha_submap_2,Indice_Alpha_submap_2) = ...
        Omega(Indice_Alpha_submap_2,Indice_Alpha_submap_2) + Omega_now(4:6,4:6);
    Xi(Indice_Alpha_submap_1) = Xi(Indice_Alpha_submap_1) + Xi_now(1:3);
    Xi(Indice_Alpha_submap_2) = Xi(Indice_Alpha_submap_2) + Xi_now(4:6);
    
        
    
    
end



end

