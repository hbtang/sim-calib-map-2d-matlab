function [traj] = relocal(odo, odoNum, mark, markNum, map_final, mapMarkNum, T_base_proj)

% do relocalization of the trajectory based on the map_final
traj = [];
T_w_odo_now = [];
for i = 1:odoNum
    vec_odo = [odo.x(i);odo.y(i);odo.theta(i)];
    stamp_odo = odo.stamp(i);
    mark_id = -1;
    T_odo_b = vec_matrix_2d(vec_odo);
    T_b_c = T_base_proj;
    T_c_m = [];
    T_w_m = [];
    for j = 1:markNum
        if stamp_odo == mark.stamp(j)
            mark_id = mark.id(j);
            vec_c_m = mark.vec2d(j,:)';
            T_c_m = vec_matrix_2d(vec_c_m);
            for k = 1:mapMarkNum
                if map_final(k).id == mark_id
                    vec_w_m = map_final(k).vec;
                    T_w_m = vec_matrix_2d(vec_w_m);
                    break;
                end
            end
            break;
        end
    end
    if norm(T_c_m) ~= 0 && norm(T_w_m) ~= 0
        T_w_odo_now = inv(T_odo_b*T_b_c*T_c_m*inv(T_w_m));
    end
    if norm(T_w_odo_now) ~= 0
        T_w_b_correct = T_w_odo_now*T_odo_b;
        vec_w_b_correct = vec_matrix_2d(T_w_b_correct);
        robotStatus.stamp =  stamp_odo;
        robotStatus.vec = vec_w_b_correct;
        traj = [traj; robotStatus];
    end
end