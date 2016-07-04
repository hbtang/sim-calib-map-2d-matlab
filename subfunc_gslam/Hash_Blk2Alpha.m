function [ vec_out ] = Hash_Blk2Alpha( HashTable_Idx, idx_blk )
%HASH_BLK2ALPHA Summary of this function goes here
%   Detailed explanation goes here

idx_tmp = find(HashTable_Idx.idx_blk == idx_blk);

idx_b = HashTable_Idx.idx_st_b(idx_tmp);
idx_e = HashTable_Idx.idx_st_e(idx_tmp);

vec_out = (idx_b:idx_e).';


end

