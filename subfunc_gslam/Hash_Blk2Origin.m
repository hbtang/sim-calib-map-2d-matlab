function [ Idx_origin ] = Hash_Blk2Origin( HashTable_Idx, Idx_blk)
%HASH_BLK2ORIGIN Summary of this function goes here
%   Detailed explanation goes here

idx_tmp = find(HashTable_Idx.idx_blk == Idx_blk);

if numel(idx_tmp) ~= 1
    error('Hash Error!!!');
else
    Idx_origin = HashTable_Idx.idx_origin(idx_tmp);
end

end

