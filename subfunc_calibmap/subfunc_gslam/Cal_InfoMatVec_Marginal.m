function [ Omega_res, Xi_res ] = Cal_InfoMatVec_Marginal( Omega, Xi, IdxVec_key )
%CAL_INFOMATVEC_MARGINAL Summary of this function goes here

[sz_Omega, tmp] = size(Omega);
[sz_Xi, tmp] = size(Xi);
if mod(sz_Omega,3) ~= 0 || mod(sz_Xi,3) ~= 0
    error('Size of matrix or vector could not be divided by 3!!!');
end

IdxVec_Alpha_key = [];
num_key = numel(IdxVec_key);
for i = (1:num_key)
    Idx_key_now = IdxVec_key(i);    
    IdxVec_tmp = (Idx_key_now*3-2 : Idx_key_now*3).';
    IdxVec_Alpha_key = [IdxVec_Alpha_key; IdxVec_tmp];    
end

IdxVec_Alpha_normal = [];
for i = 1:sz_Xi
    if numel(find(IdxVec_Alpha_key == i)) == 0 
        IdxVec_Alpha_normal = [IdxVec_Alpha_normal;i];
    end
end

% k: key nodes, n: norm nodes
Omega_kk = Omega(IdxVec_Alpha_key, IdxVec_Alpha_key);
Omega_kn = Omega(IdxVec_Alpha_key, IdxVec_Alpha_normal);
Omega_nk = Omega(IdxVec_Alpha_normal, IdxVec_Alpha_key);
Omega_nn = Omega(IdxVec_Alpha_normal, IdxVec_Alpha_normal);

Xi_k = Xi(IdxVec_Alpha_key);
Xi_n = Xi(IdxVec_Alpha_normal);

% Do marginalize
Omega_tmp = Omega_kn/Omega_nn;
Omega_res = Omega_kk - Omega_tmp*Omega_nk;
Xi_res = Xi_k - Omega_tmp*Xi_n;

Omega_res = 0.5*(Omega_res+Omega_res');

