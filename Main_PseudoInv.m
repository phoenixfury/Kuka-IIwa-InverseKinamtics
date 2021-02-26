%% Constants

link_lengths = [340, 200, 200, 200, 200, 126, 0]*1e-3;

% q_0 = [ 0.4,0.7,-0.9,-1,-.1,0.3,0];

q_0 = [1.0, 1.0, -1, -1.5, -2.53, 2.0, 2.0];

p_global = [300*1e-3 256*1e-3 500*1e-3 pi/4 pi/5 0];

%% Square trajectory
N = 10;
start = 200*1e-3;
final = 400*1e-3;
height = 550*1e-3;
points_x = linspace(start,final,N);
points_y = linspace(final,start,N);

combined1 = [points_x;final*ones(1,N);  height*ones(1,N)];
combined2 = [final*ones(1,N); points_y; height*ones(1,N)];
combined3 = [points_y; start*ones(1,N); height*ones(1,N)];
combined4 = [start*ones(1,N); points_x; height*ones(1,N)];

total_points = [combined1, combined2, combined3, combined4];
%% Visualizing the home position
[T, T1, T2, T3, T4, T5, T6] = FK_IIWA(q_0, link_lengths);

figure

phi_x = atan2(T(3,1),T(3,2));
phi_z = atan2(T(1,3),-T(2,3));
phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));

cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];

count = 1;

hold on
old = p_global;
color_list = {'blue', 'red', 'black'};
%% Getting the pseudo inverse
for counter = 1: length(total_points)
    p_global = [total_points(1,counter); total_points(2,counter); total_points(3,counter); pi/2; 0; 0];
    old = p_global;
    while norm(p_global(1:3) - cur_pos(1:3)) > 1e-05
%         [q, q_dot] = PseudoInverse(q_0, link_lengths, p_global, 0);
        %[q, q_dot] = Damped_LS(q_0, link_lengths, p_global);
        %[q, q_dot] = Null_Space(q_0, link_lengths, p_global, 0);
        [q, q_dot] = TaskAugmentation(q_0, link_lengths, p_global);
        [T, T1, T2, T3, T4, T5, T6] = FK_IIWA(q_0, link_lengths);

        if norm(old(1:3) - cur_pos(1:3)) < 1.1e-05
          Visualize_Robot(T, T1, T2, T3, T4, T5, T6, color_list{3}, 1)
          pause(0.1);
        end
%         if norm(old(1:3) - cur_pos(1:3)) > 70.1e-03
%             if norm(old - p_global) ~= 0
%               Visualize_Robot(T, T1, T2, T3, T4, T5, T6, color_list{3}, 0)
%               pause(0.1);
%             end
%             old = cur_pos;
%         end
        phi_x = atan2(T(3,1),T(3,2));
        phi_z = atan2(T(1,3),-T(2,3));
        phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
        cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];

        q_0 = q;
        count = count + 1; 
    end
end
plot3(total_points(1,:), total_points(2,:), total_points(3,:))
% figure
Visualize_Robot(T, T1, T2, T3, T4, T5, T6, 'blue', 0)

