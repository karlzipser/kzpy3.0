function f = moveto(x,y,prop)

% function f = moveto(x,y,prop)
%
% <x>,<y> are matrices
% <prop> is a proportion in [0,1]
%
% simply return x + (y-x)*prop.

f = x + (y-x)*prop;
