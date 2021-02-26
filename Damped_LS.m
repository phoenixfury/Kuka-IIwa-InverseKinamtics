function [q, q_dot] = Damped_LS(q_0, link_lengths, p_global)
%DAMPED_LS Summary of this function goes here
%   Detailed explanation goes here

deltaT = 1;
k = 1000;

u = sqrt(0.001);
%% Calculating the jacobian
J = J_IIWA(q_0, link_lengths);


%% Calculating the forward kinematics
T = FK_IIWA(q_0, link_lengths);

phi_x = atan2(T(3,1),T(3,2));
phi_z = atan2(T(1,3),-T(2,3));
phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));

cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];

%% Getting the r vector
r = p_global - cur_pos;

%% Numerical differentiation
r_dot = r./k; % To decrease the step that is taken by the velocity, k is some large constant

%% Calculate the J_inverse
J_DLS= J'/(J*J' + u^2*eye(6));

%% Caclulating the q_dot
q_dot = J_DLS * r_dot;

q = q_0+ (q_dot .* deltaT)';
end

