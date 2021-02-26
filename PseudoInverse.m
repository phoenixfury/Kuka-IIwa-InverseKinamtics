function [q, q_dot] = PseudoInverse(q_0, link_lengths, p_global, flag)
%PSEUDOINVERSE Summary of this function goes here
%   Detailed explanation goes here

deltaT = 1;
k = 1000;

weights = [1, 7500, 1, 1, 1, 1, 1];
W = diag(weights);
%% Calculating the jacobian
J = J_IIWA(q_0, link_lengths);

%J = J([1 2 3],:);
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

if flag == 1
    J_inv_ps = pinv(J);
else %Weighted pseudo inverse
   J_inv_ps =  W\J'/(J/W*J');
end
%% Caclulating the q_dot
q_dot = J_inv_ps * r_dot;

q = q_0+ (q_dot .* deltaT)';
end

