function [T, T1, T2, T3, T4, T5, T6] =  FK_IIWA(q, link_lengths)
%FK_IIWA Summary of this function goes here
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

%% Transformations
T1 = Rz(q1) * Tz(L1);

T2 = T1 * Rx(q2) * Tz(L2);

T3 = T2 * Rz(q3) * Tz(L3);

T4 = T3 * Rx(q4) * Tz(L4);

T5 = T4 * Rz(q5) * Tz(L5);

T6 = T5 * Rx(q6) * Tz(L6);

T = T6 * Rz(q7) * Tz(L7);


end

