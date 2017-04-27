function f = numreplace2(m,x,v)

% function f = numreplace2(m,x,v)
%
% <m> is a matrix
% <x> is a vector of all elements contained in <m>.
%   <x> must consist of positive integers.
% <v> (optional) is a vector of the same size as <x>
%   defaults to 1:length(x).
%
% return <m> except that each element of <m> is
% transformed into the value in <v> that corresponds
% in position to that element in <x>.
%
% here's one useful thing that you can do:
%   suppose you permute dimensions with [1 3 4 2].
%   to get the reverse permute, do:
%     f = numreplace2([1 2 3 4],[1 3 4 2]);
%   or, you could use ipermute, of course...

if ~exist('v','var')
  v = 1:length(x);
end

transform(x) = v;
f = transform(m);
