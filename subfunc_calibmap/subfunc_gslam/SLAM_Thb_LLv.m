function [ Alpha ] = SLAM_Thb_LLv( GraphDecomposeInfo, Submap_with_Bndry_Pool, Alpha, Alpha_Bnd )
%SLAM_THB_LLV Summary of this function goes here
%   Detailed explanation goes here

num_submap = max(GraphDecomposeInfo.SubmapIdx);
for i = (1:num_submap)
    Omega_tmp = Submap_with_Bndry_Pool(i).Omega_BndSub;
    Xi_tmp = Submap_with_Bndry_Pool(i).Xi_BndSub;
    IdxVec_BndUsed_tmp = Submap_with_Bndry_Pool(i).IdxVec_BndUsed;
    num_Bnd_tmp = numel(IdxVec_BndUsed_tmp);
    IdxVec_BndUsed_tmp = Trans_IdxVecNode_IdxVecAlpha(IdxVec_BndUsed_tmp);
    Alpha_k_tmp = Alpha_Bnd(IdxVec_BndUsed_tmp);
    [Omega_sub, Xi_sub] = Cal_InfoMatVec_Conditional( Omega_tmp, Xi_tmp, Alpha_k_tmp, 1:num_Bnd_tmp);
    Alpha_sub = Omega_sub\Xi_sub;
    Submap_with_Bndry_Pool(i).Alpha_Sub = Alpha_sub;
end

for i = (1:num_submap)
    IdxVec_Sub = find(GraphDecomposeInfo.SubmapIdx == i);
    IdxVec_Sub = Trans_IdxVecNode_IdxVecAlpha(IdxVec_Sub);
    Alpha(IdxVec_Sub) = Submap_with_Bndry_Pool(i).Alpha_Sub;
end
IdxVec_Bnd = find(GraphDecomposeInfo.SubmapIdx == 0);
IdxVec_Bnd = Trans_IdxVecNode_IdxVecAlpha(IdxVec_Bnd);
Alpha(IdxVec_Bnd) = Alpha_Bnd;

end

