function [ Alpha, hash_Alpha_odoIdx, hash_Alpha_markId ] = Cal_Alpha_Init( odo, mark, T_b_c )
%CAL_ALPHA_INIT Summary of this function goes here
%   Detailed explanation goes here
[sz_odo,tmp] = size(odo.stamp);
[sz_mark, tmp] = size(mark.stamp);

% Alpha: state vector
Alpha = [];
hash_Alpha_odoIdx = [];
hash_Alpha_markId = [];

Alpha = zeros(3*sz_odo, 1);
for i = 1:sz_odo
    hash_tmp = [i i];
    hash_Alpha_odoIdx = [hash_Alpha_odoIdx; hash_tmp];
    Alpha((3*i-2):(3*i)) = [odo.x(i); odo.y(i); odo.theta(i)];
end

idx_find_tmp = [];
j = 1;
for i = 1:sz_mark
    hash_tmp = [j+sz_odo mark.id(i)];
    if prod(size(hash_Alpha_markId)) ~= 0
        idx_find_tmp = find(hash_Alpha_markId(:,2) == mark.id(i));
    end
    if prod(size(idx_find_tmp)) == 0
        hash_Alpha_markId = [hash_Alpha_markId;hash_tmp];        
        odoIdx_now = mark.odoIdx(i);        
        pose_odo_b = [odo.x(odoIdx_now); odo.y(odoIdx_now); odo.theta(odoIdx_now)];
        T_odo_b = Trans_Pose_to_Mat(pose_odo_b);
        T_c_m = Trans_Pose_to_Mat(mark.vec2d(i,:)');        
        T_odo_m = T_odo_b*T_b_c*T_c_m;
        pose_odo_m = Trans_Mat_to_Pose(T_odo_m);        
        Alpha = [Alpha; pose_odo_m];
        j = j+1;
    end 
end

end

