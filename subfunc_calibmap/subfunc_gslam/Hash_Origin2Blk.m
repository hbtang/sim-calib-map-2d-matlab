function [ Idx_blk ] = Hash_Origin2Blk( HashTable_Idx, Idx_origin )
%HASH_ORIGIN2BLK Summary of this function goes here
%   find the block index from origin index

idx_tmp = find(HashTable_Idx.idx_origin == Idx_origin);

if numel(idx_tmp) ~= 1
    error('Hash Error!!!');
else
    Idx_blk = HashTable_Idx.idx_blk(idx_tmp);
end

end

