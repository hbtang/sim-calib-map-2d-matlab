function [ IdxVec_alpha ] = Trans_IdxVecNode_IdxVecAlpha( IdxVec_node )
%TRANS_IDXVECNODE_IDXVECALPHA Summary of this function goes here
%   Detailed explanation goes here
num = numel(IdxVec_node);
IdxVec_alpha = [];
for i = 1:num
    IdxVec_tmp = (IdxVec_node(i)*3-2:IdxVec_node(i)*3)';
    IdxVec_alpha = [IdxVec_alpha;IdxVec_tmp];
end
end

