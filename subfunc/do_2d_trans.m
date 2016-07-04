function [ vec_output ] = do_2d_trans( vec_input, trans_input )
%DO_2D_TRANS: do frame transform and get new vec(x,y,theta) in world frame
%   Detailed explanation goes here
% 
input_size = size(trans_input);
T1 = vec_matrix_2d(vec_input);
if input_size == [3 3]
    T = trans_input;
    T2 = T1*T;
    vec_output = vec_matrix_2d(T2);
    return;
end
if input_size == [3 1]
    T = vec_matrix_2d(trans_input);
    T2 = T1*T;
    vec_output = vec_matrix_2d(T2);
    return;
end  
error('demension of transformation matrix error!');
end

