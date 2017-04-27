function f = repeatfunc(varargin)

% function f = repeatfunc(fun,m1,m2,...,mn)
%
% <fun> is a function that accepts two arguments 
%   of the same size and returns an argument of the
%   same size
% <m1>...<mn> are matrices.  (there must be at least one.)
%
% apply <fun> repeatedly to pairs of arguments like this:
% fun(fun(fun(m1,m2),m3),m4) and so on.
%
% if there is only one argument <m1>, then just return <m1>.

% get out early
if length(varargin)==2
  f = varargin{2};
  return;
end

% do it
f = varargin{2};
for p=1:length(varargin)-2
  f = feval(varargin{1},f,varargin{p+2});
end
