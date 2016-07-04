function [ Omega, Xi, hash_mark_odo ] = SLAM_ResetOptFun( Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx,...
    odo, mark, T_b_c, sz_odo, sz_mark )
%SLAM_RENEW_INFO_ODOMETRY Summary of this function goes here

%% Create Omega and Xi
sz_Alpha = size(Alpha);
Omega = zeros(sz_Alpha(1), sz_Alpha(1));
Xi = zeros(sz_Alpha(1), 1);

%% Build Omega and Xi with odometry

idx_init = 1;
for i = idx_init:(sz_odo-1)
    hash_key = i;
    hash_key_idx = find(hash_Alpha_odoIdx(:,2) == hash_key);
    hash_value = hash_Alpha_odoIdx(hash_key_idx,1);    
    pose_1 = Alpha(3*hash_value-2 : 3*hash_value);
    pose_2 = Alpha(3*hash_value+1 : 3*hash_value+3);    
    pose_1_odo = [odo.x(i); odo.y(i); odo.theta(i)];
    pose_2_odo = [odo.x(i+1); odo.y(i+1); odo.theta(i+1)];  
    
    [Omega_now, Xi_now] = Cal_InfoMatVec_Motion(pose_1,pose_2,pose_1_odo,pose_2_odo); 
    vec_idx_1 = (3*(i-idx_init)+1):(3*(i-idx_init)+3);
    vec_idx_2 = (3*(i+1-idx_init)+1):(3*(i+1-idx_init)+3);    
    Omega(vec_idx_1, vec_idx_1) = Omega(vec_idx_1, vec_idx_1) + Omega_now(1:3,1:3);
    Omega(vec_idx_1, vec_idx_2) = Omega(vec_idx_1, vec_idx_2) + Omega_now(1:3,4:6);
    Omega(vec_idx_2, vec_idx_1) = Omega(vec_idx_2, vec_idx_1) + Omega_now(4:6,1:3);
    Omega(vec_idx_2, vec_idx_2) = Omega(vec_idx_2, vec_idx_2) + Omega_now(4:6,4:6);
    Xi(vec_idx_1) = Xi(vec_idx_1) + Xi_now(1:3);
    Xi(vec_idx_2) = Xi(vec_idx_2) + Xi_now(4:6);
end

%% Build Omega and Xi with observation

hash_mark_odo = [];
for i = 1:sz_mark
    id_m = mark.id(i);    
    id_m_alpha = Hash_Key_to_AlphaIdx( id_m, hash_Alpha_markIdx);
    id_odo = mark.odoIdx(i);
    id_odo_alpha = Hash_Key_to_AlphaIdx(id_odo, hash_Alpha_odoIdx);
    hash_mark_odo = [hash_mark_odo; id_m id_odo];
    
    vec_idx_b = 3*id_odo_alpha-2: 3*id_odo_alpha;
    vec_idx_m = 3*id_m_alpha-2: 3*id_m_alpha;
    
    pose_b = Alpha(vec_idx_b);
    pose_m = Alpha(vec_idx_m);
    pose_c_m = mark.vec2d(i,:)'; 
        
    [Omega_now, Xi_now] = Cal_InfoMatVec_Observation(pose_b, pose_m, pose_c_m, T_b_c);
    
    Omega(vec_idx_b,vec_idx_b) = Omega(vec_idx_b,vec_idx_b) + Omega_now(1:3,1:3);
    Omega(vec_idx_b,vec_idx_m) = Omega(vec_idx_b,vec_idx_m) + Omega_now(1:3,4:6);
    Omega(vec_idx_m,vec_idx_b) = Omega(vec_idx_m,vec_idx_b) + Omega_now(4:6,1:3);
    Omega(vec_idx_m,vec_idx_m) = Omega(vec_idx_m,vec_idx_m) + Omega_now(4:6,4:6);
    Xi(vec_idx_b) = Xi(vec_idx_b) + Xi_now(1:3);
    Xi(vec_idx_m) = Xi(vec_idx_m) + Xi_now(4:6);  
    

end

%% Set fixed location
% ONLY SET ODO_0 ZERO!!! Otherwise, could not converge!!!

% set odo 0
Omega(1:3,1:3) = Omega(1:3,1:3) + inv(diag([1;1;0.001]));



end

