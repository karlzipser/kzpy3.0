function f = scalerange(rg,fct)

% function f = scalerange(rg,fct)
%
% <rg> is [x y]
% <fct> is any number
%
% return the new range.

temp = (fct-1)*(rg(2)-rg(1))/2;
f(1) = rg(1) - temp;
f(2) = rg(2) + temp;
