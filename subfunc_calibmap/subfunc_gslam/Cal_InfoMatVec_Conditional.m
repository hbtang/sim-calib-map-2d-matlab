function [ Omega_out, Xi_out ] = Cal_InfoMatVec_Conditional( Omega, Xi, Alpha_k, IdxVec_k )
%CAL_INFOMATVEC_CONDITIONAL Summary of this function goes here
%   Detailed explanation goes here

num_k = numel(IdxVec_k);
num_all = numel(Xi);
IdxVec_Alpha_k = [];
for i = 1:num_k
    Idx_tmp = IdxVec_k(i);
    IdxVec_tmp = (3*Idx_tmp-2 : 3*Idx_tmp)';  
    IdxVec_Alpha_k = [IdxVec_Alpha_k;IdxVec_tmp];
end

IdxVec_Alpha_n = (1:num_all)';
IdxVec_Alpha_n(IdxVec_Alpha_k) = [];

% n: normal nodes
% k: key nodes need to be conditioned
Omega_nn = Omega(IdxVec_Alpha_n,IdxVec_Alpha_n);
Omega_nk = Omega(IdxVec_Alpha_n,IdxVec_Alpha_k);

Omega_out = Omega_nn;
Xi_out = Xi(IdxVec_Alpha_n) - Omega_nk*Alpha_k;

% debug
% Alpha_n = Omega_out\Xi_out;
% Alpha = [Alpha_n;Alpha_k];
% Alpha - Alpha_0
end

