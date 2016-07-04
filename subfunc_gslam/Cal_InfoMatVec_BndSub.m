function [ Omega_out, Xi_out, IdxVec_BndUsed ] = Cal_InfoMatVec_BndSub( ...
    GraphDecomposeInfo, Constraint_Pool, Constraint_Hash, IdxVec_Bnd, IdxVec_Submap )
% Calculate Omega and Xi for Boundary and Submap i
%   Detailed explanation goes here

IdxVec_All = [IdxVec_Bnd;IdxVec_Submap];
num_all = numel(IdxVec_All);
num_bnd = numel(IdxVec_Bnd);
num_sub = numel(IdxVec_Submap);

Omega_out = zeros(3*num_all, 3*num_all);
Xi_out = zeros(3*num_all, 1);
Constraint_Hash_BndSub = Constraint_Hash(IdxVec_All,IdxVec_All);

[rows, cols] = find(Constraint_Hash_BndSub);

num_cnstr = numel(rows);

for i = 1:num_cnstr
    Idx_Cnstr_now = Constraint_Hash_BndSub(rows(i), cols(i));    
    Omega_now = Constraint_Pool(Idx_Cnstr_now).Omega;
    Xi_now = Constraint_Pool(Idx_Cnstr_now).Xi;    
    [ Omega_out, Xi_out ] = Add_InfoMatVec( Omega_out, Xi_out, ...
        Omega_now, Xi_now, rows(i), cols(i) );   
end

IdxVec_BndUsed = [];
IdxVec_bnd_unused = [];
for i = 1:num_bnd
    if isempty(find(Constraint_Hash_BndSub(i,:))) && isempty(find(Constraint_Hash_BndSub(:,i)))
        IdxVec_bnd_unused = [IdxVec_bnd_unused; i];
    else
        IdxVec_BndUsed = [IdxVec_BndUsed; i];
    end   
end

IdxVec_bnd_unused = Trans_IdxVecNode_IdxVecAlpha(IdxVec_bnd_unused);
Omega_out(IdxVec_bnd_unused, :) = [];
Omega_out(:, IdxVec_bnd_unused) = [];
Xi_out(IdxVec_bnd_unused) = [];