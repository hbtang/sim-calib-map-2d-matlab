function [ Omega_submap_12, Xi_submap_12 ] = Cal_InfoMatVec_JointSubmap( GraphDecomposeInfo, Constraint_Pool, Constraint_Hash, ...
    Omega_submap_1, Xi_submap_1, Idx_submap_1, Omega_submap_2, Xi_submap_2, Idx_submap_2)
%CAL_INFOMATVEC_JOINTSUMAP Summary of this function goes here
%   Detailed explanation goes here

Omega_submap_12 = blkdiag(Omega_submap_1,Omega_submap_2);
Xi_submap_12 = [Xi_submap_1;Xi_submap_2];

IdxVec_Alpha_submap_1 = find(GraphDecomposeInfo.SubmapIdx == Idx_submap_1);
IdxVec_Alpha_submap_2 = find(GraphDecomposeInfo.SubmapIdx == Idx_submap_2);

Constraint_Hash_submap_12 = Constraint_Hash(IdxVec_Alpha_submap_1,IdxVec_Alpha_submap_2);
[rowVec_12, colVec_12] = find(Constraint_Hash_submap_12);
if numel(rowVec_12)~= 0
    for i = numel(rowVec_12)
        Idx_ConstrPool_now = Constraint_Hash_submap_12(rowVec_12(i), colVec_12(i));
        AlphaIdx_1 = Constraint_Pool(Idx_ConstrPool_now).AlphaIdx_1;
        AlphaIdx_2 = Constraint_Pool(Idx_ConstrPool_now).AlphaIdx_2;
        
        LocalIdx_1 = GraphDecomposeInfo.LocalIdx(AlphaIdx_1);
        LocalIdx_2 = GraphDecomposeInfo.LocalIdx(AlphaIdx_2);
        
        JointIdx_1 = LocalIdx_1;
        JointIdx_2 = LocalIdx_2 + numel(Xi_submap_1)/3;
        
        Omega_submap_now = Constraint_Pool(Idx_ConstrPool_now).Omega;
        Xi_submap_now = Constraint_Pool(Idx_ConstrPool_now).Xi;
        
        [ Omega_submap_12, Xi_submap_12 ] = Add_InfoMatVec( Omega_submap_12, Xi_submap_12, Omega_submap_now, Xi_submap_now, ...
            JointIdx_1, JointIdx_2 );
    end
end

Constraint_Hash_submap_21 = Constraint_Hash(IdxVec_Alpha_submap_2,IdxVec_Alpha_submap_1);
[rowVec_21, colVec_21] = find(Constraint_Hash_submap_21);

if numel(rowVec_21) ~= 0
    for i = numel(rowVec_21)
        Idx_ConstrPool_now = Constraint_Hash_submap_21(rowVec_21(i), colVec_21(i));
        AlphaIdx_1 = Constraint_Pool(Idx_ConstrPool_now).AlphaIdx_1;
        AlphaIdx_2 = Constraint_Pool(Idx_ConstrPool_now).AlphaIdx_2;
        
        LocalIdx_1 = GraphDecomposeInfo.LocalIdx(AlphaIdx_1);
        LocalIdx_2 = GraphDecomposeInfo.LocalIdx(AlphaIdx_2);
        
        JointIdx_1 = LocalIdx_1 + numel(Xi_submap_1)/3;
        JointIdx_2 = LocalIdx_2;
        
        Omega_submap_now = Constraint_Pool(Idx_ConstrPool_now).Omega;
        Xi_submap_now = Constraint_Pool(Idx_ConstrPool_now).Xi;
        
        [ Omega_submap_12, Xi_submap_12 ] = Add_InfoMatVec( Omega_submap_12, Xi_submap_12, Omega_submap_now, Xi_submap_now, ...
            JointIdx_2, JointIdx_1 );
    end
end

end

