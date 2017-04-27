function f = evib(x)

% function f = evib(x)
%
% <x> is a matrix or a string referring to a variable
%   in the base workspace.
%
% evib stands for "evalulate in base if necessary".
% if <x> is a string, eval it in the base workspace
% and return the result.  if <x> isn't a string,
% just return it.

if ischar(x)
  f = evalin('base',x);
else
  f = x;
end
