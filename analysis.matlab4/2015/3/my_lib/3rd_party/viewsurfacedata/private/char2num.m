function f = char2num(m)

% function f = char2num(m)
%
% <m> is a character array
%
% transform each element of <m> according to the following:
% '!@#$%^&*()' maps to 1 through 10.

f = numreplace2(double(m),[33    64    35    36    37    94    38    42    40    41]);
