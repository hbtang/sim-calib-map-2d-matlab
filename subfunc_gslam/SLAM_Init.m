function [ Alpha, Omega, Xi, hash_Alpha_odoIdx, hash_Alpha_markIdx ] = SLAM_Init( odo, mark, T_b_c )
%SLAM_INIT Summary of this function goes here
% Init SLAM algorith
% Create information matrix: Omega
% Create information vector: Xi
% Create state vector: Alpha
% Create hash table: hash_Alpha_odoIdx, hash_Alpha_markIdx

[ Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx ] = Cal_Alpha_Init( odo, mark, T_b_c );
sz_Alpha = size(Alpha);
Omega = zeros(sz_Alpha(1), sz_Alpha(1));
Xi = zeros(sz_Alpha(1), 1);

end

