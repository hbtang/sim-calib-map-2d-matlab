function [ Constraint_Pool, Constraint_Hash ] = Create_Constraint_Pool( Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx, odo, mark, sz_odo, sz_mark, T_b_c )
%   CREATE_CONSTRAINT_POOL: Create constraint pool with Omega and Xi of all
%   the constraints, and also Hash table from index to the information

Constraint_tmp = struct('Omega',[],'Xi',[],'AlphaIdx_1',[],'AlphaIdx_2',[]);
Constraint_Pool(sz_odo+sz_mark-1,1) = Constraint_tmp;

num_nodes = numel(Alpha)/3;
Constraint_Hash = zeros(num_nodes,num_nodes);

for i = 1:(sz_odo-1)
    hash_key = i;
    hash_key_idx = find(hash_Alpha_odoIdx(:,2) == hash_key);
    hash_value = hash_Alpha_odoIdx(hash_key_idx,1);
    AlphaIdx_1 = hash_value;
    AlphaIdx_2 = hash_value+1;
    pose_1 = Alpha(3*AlphaIdx_1-2 : 3*AlphaIdx_1);
    pose_2 = Alpha(3*AlphaIdx_2-2 : 3*AlphaIdx_2);
    
    pose_1_odo = [odo.x(i); odo.y(i); odo.theta(i)];
    pose_2_odo = [odo.x(i+1); odo.y(i+1); odo.theta(i+1)];  
    
    [Omega_now, Xi_now] = Cal_InfoMatVec_Motion(pose_1,pose_2,pose_1_odo,pose_2_odo); 
    
    Constraint_tmp.Omega = Omega_now;
    Constraint_tmp.Xi = Xi_now;
    Constraint_tmp.AlphaIdx_1 = AlphaIdx_1;
    Constraint_tmp.AlphaIdx_2 = AlphaIdx_2;
    
    Constraint_Pool(i,1) = Constraint_tmp;    
    Constraint_Hash(Constraint_tmp.AlphaIdx_1,Constraint_tmp.AlphaIdx_2) = i;
end

for i = 1:sz_mark
    id_odo = mark.odoIdx(i);
    AlphaIdx_b = Hash_Key_to_AlphaIdx(id_odo, hash_Alpha_odoIdx);
    id_m = mark.id(i);    
    AlphaIdx_m = Hash_Key_to_AlphaIdx( id_m, hash_Alpha_markIdx);
    
    vec_idx_b = 3*AlphaIdx_b-2: 3*AlphaIdx_b;
    vec_idx_m = 3*AlphaIdx_m-2: 3*AlphaIdx_m;
    
    pose_b = Alpha(vec_idx_b);
    pose_m = Alpha(vec_idx_m);
    pose_c_m = mark.vec2d(i,:)'; 
    
    [Omega_now, Xi_now] = Cal_InfoMatVec_Observation(pose_b, pose_m, pose_c_m, T_b_c);
    
    Constraint_tmp.Omega = Omega_now;
    Constraint_tmp.Xi = Xi_now;
    Constraint_tmp.AlphaIdx_1 = AlphaIdx_b;
    Constraint_tmp.AlphaIdx_2 = AlphaIdx_m;
    Constraint_Pool(i+sz_odo-1,1) = Constraint_tmp;   
    Constraint_Hash(Constraint_tmp.AlphaIdx_1,Constraint_tmp.AlphaIdx_2) = i+sz_odo-1;
end

end

