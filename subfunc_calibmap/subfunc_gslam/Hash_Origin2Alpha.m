function [ vec_out ] = Hash_Origin2Alpha( HashTable_Idx, Idx_origin )
%HASH_ORIGIN2BLK Summary of this function goes here
%   find the block index from origin index

idx_tmp = find(HashTable_Idx.idx_origin == Idx_origin);

if numel(idx_tmp) ~= 1
    error('Hash Error!!!');
else
    idx_b = HashTable_Idx.idx_st_b(idx_tmp);
    idx_e = HashTable_Idx.idx_st_e(idx_tmp);
    vec_out = (idx_b:idx_e).';
end

end

