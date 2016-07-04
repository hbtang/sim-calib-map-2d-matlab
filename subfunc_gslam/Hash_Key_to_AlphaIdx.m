function [ value ] = Hash_Key_to_AlphaIdx( key, Hash )
%HASH_KEY_TO_ALPHAIDX Summary of this function goes here
%   Detailed explanation goes here
    hash_key_idx = find(Hash(:,2) == key);
    if(numel(hash_key_idx)) ~= 0 
        value = Hash(hash_key_idx,1);
    else
        value = NULL;
    end
end

