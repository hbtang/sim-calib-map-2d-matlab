%% display to screen
format short g;
display('--------------final result-----------------');
display('Ground plane w.r.t. cam frame expressed by homogeneous vector:');
display(vec_ground);
display('Transformation matrix from cam frame to projective frame on the ground:');
display(T_cam_proj);
display('Camera extrinsic parameter 2d calibration result:');
display(T_base_proj);
map_matrix = [];
for i = 1:mapMarkNum
    id = map(i).id;
    x = map(i).vec(1);
    y = map(i).vec(2);
    theta = map(i).vec(3);
    map_matrix = [map_matrix; [id x y theta]];
end
% display('Map displayed in matrix form, [id x y theta]');
% display(map_matrix);
vec_id = map_matrix(:,1);
map_matrix_pure = map_matrix(:,2:4);
display('Map id vec:')
display(vec_id);
display('Map matrix: [x y theta] for each row')
display(map_matrix_pure);

%% Change world frame into a special case for experient in HIT

theta_map2standard_HIT = atan2(-map_matrix_pure(4,2), -map_matrix_pure(4,1));
for i = 1:10
    map_matrix_pure(i,3) = map_matrix_pure(i,3)-theta_map2standard_HIT;
    R_map2standard_HIT = [cos(theta_map2standard_HIT) -sin(theta_map2standard_HIT);...
        sin(theta_map2standard_HIT) cos(theta_map2standard_HIT)];
    vec_tmp = R_map2standard_HIT.'*map_matrix_pure(i,1:2).';
    map_matrix_pure(i,1:2) = vec_tmp.';
end
display(map_matrix_pure);


%% write into file
outputFilePath = ['data/thb_local_record_' dataSerialNum '/thb_calib_map_' dataSerialNum '.yml'];
outputFileId = fopen(outputFilePath,'w');
fprintf(outputFileId,'%%YAML:1.0\n');
writeYmlMatrix( outputFileId, T_cam_proj, 'T_cam3d_cam2d', 'f');
writeYmlMatrix( outputFileId, T_base_proj, 'T_base_cam', 'f');
writeYmlMatrix( outputFileId, vec_id, 'vec_id', 'd');
writeYmlMatrix( outputFileId, map_matrix_pure, 'map_matrix', 'f');
fclose(outputFileId);