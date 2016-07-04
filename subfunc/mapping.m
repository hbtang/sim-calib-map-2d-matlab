function [map_final, mapMarkNum] = mapping(mark, markNum, odo, T_base_proj, idNum)

%% calculate constraints between marks
constraints_pool = [];
for i = 1:(markNum-1)
    if mark.id(i) ~= mark.id(i+1)
        % frame defined as:
        % c: camera
        % m: marker
        % b: base
        
        stamp1 = mark.stamp(i);
        stamp2 = mark.stamp(i+1);
        theta_w_b1 = odo.theta(stamp1);
        theta_w_b2 = odo.theta(stamp2);
        x_w_b1 = odo.x(stamp1);
        x_w_b2 = odo.x(stamp2);
        y_w_b1 = odo.y(stamp1);
        y_w_b2 = odo.y(stamp2);
        T_w_b1 = cal_2d_trans_matrix(x_w_b1,y_w_b1,theta_w_b1);
        T_w_b2 = cal_2d_trans_matrix(x_w_b2,y_w_b2,theta_w_b2);
        
        x_c1_m1 = mark.vec2d(i,1);
        y_c1_m1 = mark.vec2d(i,2);
        theta_c1_m1 = mark.vec2d(i,3);
        T_c1_m1 = cal_2d_trans_matrix(x_c1_m1,y_c1_m1,theta_c1_m1);
        
        x_c2_m2 = mark.vec2d(i+1,1);
        y_c2_m2 = mark.vec2d(i+1,2);
        theta_c2_m2 = mark.vec2d(i+1,3);
        T_c2_m2 = cal_2d_trans_matrix(x_c2_m2,y_c2_m2,theta_c2_m2);
        
        T_m1_m2 = inv(T_c1_m1)*inv(T_base_proj)*inv(T_w_b1)...
            *T_w_b2*T_base_proj*T_c2_m2;
        id1 = mark.id(i);
        id2 = mark.id(i+1);
        constraint_temp.id1 = id1;
        constraint_temp.id2 = id2;
        constraint_temp.T = T_m1_m2;
        constraint_temp.vec = [T_m1_m2(1,3);T_m1_m2(2,3);atan2(T_m1_m2(2,1),T_m1_m2(1,1))];
        constraints_pool = [constraints_pool; constraint_temp];
    end
end

%% generate map by optimization algorithm

temp = size(constraints_pool);
constrNum = temp(1);
% idNum = 10;

H_opt = zeros(idNum*3);
f_opt = zeros(idNum*3,1);

%% convariance matrix of landmark constraints
% set std_x std_y and std_theta according to the platform
std_x = 20; % 20 mm
std_y = 20;
std_theta = 1*pi/180; % 3 degree

Conv_opt = diag([std_x^2;std_y^2;std_theta^2]);
Inf_opt = inv(Conv_opt);

%% initial guess
vec_opt_init = zeros(3*idNum,1);
if_tag_init = zeros(idNum,1); % from id0 to id3
if_tag_init(1) = 1;
for i = 1:constrNum
    T_temp = constraints_pool(i).T;
    id1_temp = constraints_pool(i).id1;
    id2_temp = constraints_pool(i).id2;
    if if_tag_init(id2_temp+1) == 0
        vec_temp_1 = vec_opt_init(3*id1_temp+1:3*id1_temp+3);
        vec_temp_2 = do_2d_trans(vec_temp_1,T_temp);
        vec_opt_init(3*id2_temp+1 : 3*id2_temp+3) = vec_temp_2;
        if_tag_init(id2_temp+1) = 1;
    end
    if if_tag_init == ones(idNum,1);
        break;
    end
end

%% debug: display init map
map_init = [];
for i = 0:idNum-1
    landmark.id = i;
    span_i = 3*i+1:3*i+3;
    landmark.vec = [vec_opt_init(span_i)];
    map_init = [map_init;landmark];
end

%% start optimization
vec_opt = vec_opt_init;
ifConverged = 0;
loopCount_opt = 0;
while ~ifConverged
    H = zeros(3*idNum,3*idNum);
    b = zeros(3*idNum,1);
    f_opt = 0; % value of destination function
    for i = 1:constrNum
        Oij = Inf_opt;
        idi_temp = constraints_pool(i).id1;
        idj_temp = constraints_pool(i).id2;
        vec_i_temp = vec_opt(3*idi_temp+1:3*idi_temp+3);
        vec_j_temp = vec_opt(3*idj_temp+1:3*idj_temp+3);
        [Aij Bij eij] = cal_AB_opt(vec_i_temp,vec_j_temp,constraints_pool(i).T);
        span_i = 3*idi_temp+1:3*idi_temp+3;
        span_j = 3*idj_temp+1:3*idj_temp+3;
        H(span_i,span_i) = H(span_i,span_i) + Aij.'*Oij*Aij;
        H(span_j,span_j) = H(span_j,span_j) + Bij.'*Oij*Bij;
        H(span_i,span_j) = H(span_i,span_j) + Aij.'*Oij*Bij;
        H(span_j,span_i) = H(span_j,span_i) + Bij.'*Oij*Aij;
        b(span_i) = b(span_i) + Aij'*Oij*eij;
        b(span_j) = b(span_j) + Bij'*Oij*eij;
        f_opt = f_opt + eij'*Oij*eij;
    end
    Aeq = zeros(3*idNum,3*idNum);
    Aeq(1:3,1:3) = eye(3);
    beq = zeros(3*idNum,1);
    dvec_opt = quadprog(H,b,[],[],Aeq,beq);
    vec_opt = vec_opt+dvec_opt;
    if loopCount_opt > 1000
        display('Opt run over maximum times!');
        break;
    end
    if norm(dvec_opt) < 0.001
        ifConverged = 1;
    end
    %     f_opt
    loopCount_opt = loopCount_opt+1;
end

%% generate map
% each row for one tag
map_final = [];
for i = 0:idNum-1
    landmark.id = i;
    span_i = 3*i+1:3*i+3;
    landmark.vec = [vec_opt(span_i)];
    map_final = [map_final;landmark];
end

%% set mapMarkNum
mapMarkNum = size(map_final);
mapMarkNum = mapMarkNum(1);

%% rectify map: only for HIT set up
vec_w_m0 = map_final(1).vec;
vec_w_m3 = map_final(4).vec;
dist_m0_m3 = norm(vec_w_m0(1:2)-vec_w_m3(1:2));
dist_m0_m3_measure = 14400;
map_ratio_tmp = dist_m0_m3_measure/dist_m0_m3;
alpha_m0_m3 = atan2(vec_w_m3(2)-vec_w_m0(2),vec_w_m3(1)-vec_w_m0(1));
d_alpha = alpha_m0_m3 + pi/2;
for i = 1:mapMarkNum
    map_final(i).vec(1:2) = map_ratio_tmp*map_final(i).vec(1:2);
    vec_w_wnew = [0;0;d_alpha];
    T_w_wnew = vec_matrix_2d(vec_w_wnew);
    vec_w_m = map_final(i).vec;
    T_w_m = vec_matrix_2d(vec_w_m);
    T_wnew_m = inv(T_w_wnew)*T_w_m;  
    vec_wnew_m = vec_matrix_2d(T_wnew_m);   
    map_final(i).vec = vec_wnew_m;
end

