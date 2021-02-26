function [q, q_dot] = TaskAugmentation(q_0, link_lengths, p_global)
%TASKAUGMENTATION Summary of this function goes here
%   Detailed explanation goes here
deltaT = 1;
k = 100;
u = sqrt(0.001);
%% Calculating the jacobian
J = J_IIWA(q_0, link_lengths);

J1 = J(1:3, :);

J2 = J(4:5, :);

%% Calculating the forward kinematics
T = FK_IIWA(q_0, link_lengths);

phi_x = atan2(T(3,1),T(3,2));
phi_z = atan2(T(1,3),-T(2,3));
phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));

cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];

%% Getting the r vector
r1 = p_global(1:3) - cur_pos(1:3);

r2 = p_global(4:5) - cur_pos(4:5);
%% Numerical differentiation
r_dot1 = r1./k; % To decrease the step that is taken by the velocity, k is some large constant

r_dot2 = r2./k;
%% Calculate the J_inverse
J1_hash = (J1'/(J1*J1' + u^2*eye(3)));

I = eye(7);

P1 = (I - J1_hash*J1);

J2_hash = ((J2 * P1)'/((J2 * P1)*(J2 * P1)' + u^2*eye(2)));

v1 = J2_hash * (r_dot2 - J2*J1_hash*r_dot1);

%% Caclulating the q_dot
q_dot = (J1_hash * r_dot1) + (P1 * v1);

q = q_0+ (q_dot .* deltaT)';


end

