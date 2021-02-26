function [J, J1, J2, J3, J4] =  J_IIWA(q, link_lengths)
%J_IIWA Summary of this function goes here
%% Extracting the link lengths
L1 = link_lengths(1);
L2 = link_lengths(2);
L3 = link_lengths(3);
L4 = link_lengths(4);
L5 = link_lengths(5);
L6 = link_lengths(6);
L7 = link_lengths(7);

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);
%% Getting the forward kinematics

T =  FK_IIWA(q, link_lengths);

T(1:3, 4) = 0;

%% Getting the jacobians

Td = Rzd(q1) * Tz(L1)* Rx(q2) * Tz(L2)* Rz(q3) * Tz(L3)* Rx(q4) * Tz(L4)* Rz(q5) * Tz(L5)* Rx(q6) * Tz(L6)* Rz(q7) * Tz(L7) / T;
 
J1 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Rz(q1) * Tz(L1)* Rxd(q2) * Tz(L2)* Rz(q3) * Tz(L3)* Rx(q4) * Tz(L4)* Rz(q5) * Tz(L5)* Rx(q6) * Tz(L6)* Rz(q7) * Tz(L7) / T;
 
J2 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Rz(q1) * Tz(L1)* Rx(q2) * Tz(L2)* Rzd(q3) * Tz(L3)* Rx(q4) * Tz(L4)* Rz(q5) * Tz(L5)* Rx(q6) * Tz(L6)* Rz(q7) * Tz(L7) / T;
 
J3 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Rz(q1) * Tz(L1)* Rx(q2) * Tz(L2)* Rz(q3) * Tz(L3)* Rxd(q4) * Tz(L4)* Rz(q5) * Tz(L5)* Rx(q6) * Tz(L6)* Rz(q7) * Tz(L7) / T;
 
J4 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Rz(q1) * Tz(L1)* Rx(q2) * Tz(L2)* Rz(q3) * Tz(L3)* Rx(q4) * Tz(L4)* Rzd(q5) * Tz(L5)* Rx(q6) * Tz(L6)* Rz(q7) * Tz(L7) / T;
 
J5 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Rz(q1) * Tz(L1)* Rx(q2) * Tz(L2)* Rz(q3) * Tz(L3)* Rx(q4) * Tz(L4)* Rz(q5) * Tz(L5)* Rxd(q6) * Tz(L6)* Rz(q7) * Tz(L7) / T;
 
J6 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Rz(q1) * Tz(L1)* Rx(q2) * Tz(L2)* Rz(q3) * Tz(L3)* Rx(q4) * Tz(L4)* Rz(q5) * Tz(L5)* Rx(q6) * Tz(L6)* Rzd(q7) * Tz(L7) / T;
 
J7 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

J = [J1, J2, J3, J4, J5, J6, J7];
end

