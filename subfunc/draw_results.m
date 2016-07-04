function [] = draw_results(mark,markNum,odo,odoNum,traj,map,mapMarkNum,T_base_proj)

%% draw results

% clf;
figure;
set(gca, 'xlim', [-4000 8000]);
set(gca, 'ylim', [-20000 4000]);
set(gca,'XTick',-60000:600:60000);
set(gca,'YTick',-60000:600:60000);
set(gca,'DataAspectRatio',[1 1 1]);
set(gca,'fontsize', 10);
grid on;
hold on;



% set draw config

markSize = 600;

pt_static = [
    0.5*markSize 0.5*markSize 1;
    0.5*markSize -0.5*markSize 1;
    -0.5*markSize -0.5*markSize 1;
    -0.5*markSize 0.5*markSize 1;
    ]';
pt_axis_static = [
    0 0 1;
    1*markSize 0 1;
    0 0 1;
    0 1*markSize 1;
    ]';

%% draw odo. traj.
% Calculate transformation matrix from odo to global (landmark 0)
vec_c0_m0 = mark.vec2d(1,:)';
vec_b_c = vec_matrix_2d(T_base_proj);
vec_b0_m0 = do_2d_trans(vec_b_c,vec_c0_m0);
vec_odo0_b0 = [
    odo.x(mark.stamp(1));
    odo.y(mark.stamp(1));
    odo.theta(mark.stamp(1));
    ];
T_b0_m0 = vec_matrix_2d(vec_b0_m0);
T_odo0_b0 = vec_matrix_2d(vec_odo0_b0);
T_odo0_w = T_odo0_b0*T_b0_m0;
for i = 1:(odoNum-1)
    stamp_temp = i;
    vec_odo_b1 = [odo.x(stamp_temp);odo.y(stamp_temp);odo.theta(stamp_temp)];
    T_odo_b1 = vec_matrix_2d(vec_odo_b1);
    T_w_b1 = inv(T_odo0_w)*T_odo_b1;
    vec_odo_b2 = [odo.x(stamp_temp+1);odo.y(stamp_temp+1);odo.theta(stamp_temp+1)];
    T_odo_b2 = vec_matrix_2d(vec_odo_b2);
    T_w_b2 = inv(T_odo0_w)*T_odo_b2;
    % draw odo
    pt1 = T_w_b1*[0;0;1];
    pt2 = T_w_b2*[0;0;1];
    pt3 = T_w_b2*[50;0;1];
    line([pt1(1);pt2(1)],[pt1(2);pt2(2)],'Color','b','LineStyle','.','LineWidth',2);
    %     line([pt1(1);pt3(1)],[pt1(2);pt3(2)],'Color','r');
end

%% draw traj. corrected
trajNum = size(traj);
trajNum = trajNum(1);
for i = 1:(trajNum-1)
    stamp_temp = i;
    vec_w_b1 = traj(i).vec;
    vec_w_b2 = traj(i+1).vec;
    T_w_b1 = vec_matrix_2d(vec_w_b1);
    T_w_b2 = vec_matrix_2d(vec_w_b2);
    % draw odo
    pt1 = T_w_b1*[0;0;1];
    pt2 = T_w_b2*[0;0;1];
    pt3 = T_w_b2*[50;0;1];
    line([pt1(1);pt2(1)],[pt1(2);pt2(2)],'Color','r','LineStyle','.','LineWidth',2);
    %     line([pt1(1);pt3(1)],[pt1(2);pt3(2)],'Color','r');
end

%% draw landmarks
%draw raw landmarks
for i = 1:markNum
    
    vec_c_m = mark.vec2d(i,:)';
    T_c_m = vec_matrix_2d(vec_c_m);
    T_b_m = T_base_proj*T_c_m;
    stamp = mark.stamp(i);
    vec_odo_b = [odo.x(stamp);odo.y(stamp);odo.theta(stamp)];
    T_odo_b = vec_matrix_2d(vec_odo_b);
    T_w_b = inv(T_odo0_w)*T_odo_b;
    T_w_m = inv(T_odo0_w)*T_odo_b*T_b_m;
    pt_b_center = T_w_b*[0;0;1];
    pt_mark_axis = T_w_m*pt_axis_static;
    % draw observation direction
%     line([pt_mark_axis(1,1);pt_b_center(1)],...
%         [pt_mark_axis(2,1);pt_b_center(2)],...
%         'Color','y','LineStyle','-','LineWidth',0.5);    
    line(pt_mark_axis(1,:),pt_mark_axis(2,:),'Color','g','LineWidth',1);
end

%% draw landmark in final map
for i = 1:mapMarkNum
    T = cal_2d_trans_matrix(map(i).vec(1),map(i).vec(2),map(i).vec(3));
    pt_mark = T*pt_static;
    pt_mark_axis = T*pt_axis_static;
    line(pt_mark_axis(1,:),pt_mark_axis(2,:),'Color','k','LineWidth',3);
%     if i == 1
%         fill(pt_mark(1,:),pt_mark(2,:),'r');
%     else
%         fill(pt_mark(1,:),pt_mark(2,:),'b');
%     end
end

%% draw map of ground truth
map_real = map;
map_real(1).vec = [0;0;0];
map_real(2).vec = [0;-4200;0];
map_real(3).vec = [0;-8400;0];
map_real(4).vec = [0;-14400;0];
map_real(5).vec = [3600;-14400;0];
map_real(6).vec = [6600;-8400;0];
map_real(7).vec = [6600;-4200;0];
map_real(8).vec = [6600;0;0];
map_real(9).vec = [3600;0;0];
map_real(10).vec = [3600;-6600;0];
for i = 1:mapMarkNum
    T = cal_2d_trans_matrix(map_real(i).vec(1),map_real(i).vec(2),map_real(i).vec(3));
    pt_mark = T*pt_static;
    pt_mark_axis = T*pt_axis_static;
    line(pt_mark_axis(1,:),pt_mark_axis(2,:),'Color','r','LineWidth',3);
%     if i == 1
%         fill(pt_mark(1,:),pt_mark(2,:),'r');
%     else
%         fill(pt_mark(1,:),pt_mark(2,:),'b');
%     end
end

%% draw init landmarks
% for i = 1:mapMarkNum
%     T = cal_2d_trans_matrix(map_init(i).vec(1),map_init(i).vec(2),map_init(i).vec(3));
%     pt_mark = T*pt_static;
%     pt_mark_axis = T*pt_axis_static;
%     line(pt_mark_axis(1,:),pt_mark_axis(2,:),'Color','g');
%     if i == 1
%         fill(pt_mark(1,:),pt_mark(2,:),'r');
%     else
%         fill(pt_mark(1,:),pt_mark(2,:),'g');
%     end
% end