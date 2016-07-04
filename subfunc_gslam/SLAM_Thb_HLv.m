function [ Submap_with_Bndry_Pool, Alpha_Bnd ] = ...
    SLAM_Thb_HLv( GraphDecomposeInfo, Constraint_Pool, Constraint_Hash)
%SLAM_THB_HLV Summary of this function goes here
%   Detailed explanation goes here
    num_submap = max(GraphDecomposeInfo.SubmapIdx);
    num_bnd = numel(find(GraphDecomposeInfo.SubmapIdx == 0));
    Omega_Bnd = zeros(num_bnd*3, num_bnd*3);
    Xi_Bnd = zeros(num_bnd*3, 1); 

    s_tmp = struct('Omega_BndSub',[],'Xi_BndSub',[],'IdxVec_BndUsed',[],'Alpha_Sub',[]);
    Submap_with_Bndry_Pool(num_submap,1) = s_tmp;
    
    IdxVec_Bnd = find(GraphDecomposeInfo.SubmapIdx == 0);
    for i = (1:num_submap)
        
        IdxVec_Submap = find(GraphDecomposeInfo.SubmapIdx == i); 
        
        [ Omega_BndSub, Xi_BndSub, IdxVec_BndUsed ] = Cal_InfoMatVec_BndSub( ...
            GraphDecomposeInfo, Constraint_Pool, Constraint_Hash, IdxVec_Bnd, IdxVec_Submap);
        [ Omega_Bnd_add, Xi_Bnd_add ] = Cal_InfoMatVec_Marginal( Omega_BndSub, Xi_BndSub, 1:numel(IdxVec_BndUsed));       
        
        s_tmp.Omega_BndSub = Omega_BndSub;
        s_tmp.Xi_BndSub = Xi_BndSub;
        s_tmp.IdxVec_BndUsed = IdxVec_BndUsed;        
        Submap_with_Bndry_Pool(i) = s_tmp;
        
        IdxVec_tmp = Trans_IdxVecNode_IdxVecAlpha(IdxVec_BndUsed);
        Omega_Bnd(IdxVec_tmp,IdxVec_tmp) = Omega_Bnd(IdxVec_tmp,IdxVec_tmp) + Omega_Bnd_add;
        Xi_Bnd(IdxVec_tmp) = Xi_Bnd(IdxVec_tmp) + Xi_Bnd_add;
    end  
    
    % set first node as world frame origin
    Omega_Bnd(1:3,1:3) = Omega_Bnd(1:3,1:3) + inv(diag([1;1;0.001]));   
    
    Alpha_Bnd = Omega_Bnd\Xi_Bnd; 

end

