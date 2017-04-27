function f = xyzflip(v)

% function f = xyzflip(v)
%
% return the matrix for flipping according to <v>
% which is [A B C] where each of A, B, and C are 0 or 1.

v = 2*(.5-v);

f = [v(1)    0    0 0 ;...
     0    v(2)    0 0 ;...
     0       0 v(3) 0 ;...
     0       0    0 1 ];
