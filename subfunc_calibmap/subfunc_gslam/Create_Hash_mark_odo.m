function [ hash_mark_odo ] = Create_Hash_mark_odo( mark, hash_Alpha_markIdx, hash_Alpha_odoIdx )
%CREATE_HASH_MARK_ODO Summary of this function goes here
%   Detailed explanation goes here
hash_mark_odo = [];
sz_mark = numel(mark.stamp);
for i = 1:sz_mark
    id_m = mark.id(i);    
    id_m_alpha = Hash_Key_to_AlphaIdx( id_m, hash_Alpha_markIdx);
    id_odo = mark.odoIdx(i);
    id_odo_alpha = Hash_Key_to_AlphaIdx(id_odo, hash_Alpha_odoIdx);
    hash_mark_odo = [hash_mark_odo; id_m id_odo];
end

end

