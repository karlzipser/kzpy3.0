function f = matrixinterpn(m,pts,method,offset,mode)

% function f = matrixinterpn(m,pts,method,offset,mode)
%
% <m> is an N-dimensional matrix
% <pts> is a cell vector of N matrices.  these N matrices indicate the
%   N-dimensional coordinates of the vertices which we want to interpolate
%   to find values at.
% <method> (optional) is the interpolation method:
%   'linear' | 'cubic' | 'nearest' | 'spline'
%   if [] or not supplied, default to 'linear'.
% <offset> (optional) is an N-dimensional vector indicating how much
%   extra offset was given to <m>.  if [] or not supplied, default to
%   zeros(1,N), which indicates that the first element of <m> starts
%   at (1,1,...,1).
% <mode> (optional) is
%   0 means pad <m> with a single layer of zeros around all edges
%   1 means pad <m> by repeating edges to create an extra single layer
%     around all edges.
%   2 means don't pad.
%   if [] or not supplied, default to 0.
%
% first, pad <m> according to <mode>.
% then, interpolate to find values at the points described in
% <pts> using method <method>.  return a matrix of the interpolated
% values.  it should have dimensions identical to one of the elements
% of <pts>.
%
% the <offset> parameter is a convenient way to specify a translation
% shift for <m>.  the length of <offset> should match the length
% of <pts>.
%
% note that the length of <pts> determines N, which is the
% effective dimensionality of <m>.
%
% note that <m> can be complex, in which case the real and imaginary
% arguments are separately interpolated.  this is consistent with
% the behavior of the built-in interpn routine.

% calc
n = length(pts);
msize = sizefull(m,n);

% deal with input
if ~exist('method','var') || isempty(method)
  method = 'linear';
end
if ~exist('offset','var') || isempty(offset)
  offset = zeros(1,n);
end
if ~exist('mode','var') || isempty(mode)
  mode = 0;
end

% tweak pts
for p=1:n
  pts{p} = pts{p} - offset(p) + choose(mode==2,0,1);  % compensate for offset (if necessary), then compensate for padding
end

% do it
switch mode
case 0
  f = interpn(matrixpad2(m,msize+2,'center',0),pts{:},method);
case 1
  f = interpn(matrixpad3(m,n),pts{:},method);
case 2
  f = interpn(m,pts{:},method);
end
