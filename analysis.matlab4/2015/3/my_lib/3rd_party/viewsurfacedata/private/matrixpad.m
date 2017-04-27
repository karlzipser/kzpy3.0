function f = matrixpad(m,dim,beginz,endz,what)

% function f = matrixpad(m,dim,beginz,endz,what)
% 
% <m> is a matrix.
% <dim> is a (nonempty) vector of the dimensions we want to pad.
% <beginz> is a vector of numbers telling us how much to pad at the
%   beginning (any integer).  can also be scalar.
% <endz> is a vector of numbers telling us how much to pad at the
%   end (any integer).  can also be scalar.
% <what> (optional) is the element that we use for padding
%   (if we need it).  if [] or not supplied, default to 0.
%
% this routine is potentially slow.

% deal with input
if ~exist('what','var')
  what = [];
end
if isempty(what)
  what = 0;
end
if length(beginz)==1
  beginz = repmat(beginz,1,length(dim));
end
if length(endz)==1
  endz = repmat(endz,1,length(dim));
end

% define msize
if ~isempty(m)
  msize = ones(1,max([ndims(m) dim]));
  msize(1:ndims(m)) = size(m);
else
  msize = zeros(1,max([dim 2]));  % hack.  at least two zeros to be a real size.
end

% define newmsize
newmsize = msize;
newmsize(dim) = newmsize(dim) + beginz + endz;

% define sm and sf
sm = repmat({':'},1,length(msize));
sf = sm;
for p=1:length(dim)
  sm{dim(p)} = choose(beginz(p)<0,          1-beginz(p),            1)  : ...
               choose(endz(p)<0,  msize(dim(p))+endz(p),msize(dim(p)));
  sf{dim(p)} = choose(beginz(p)>=0,            1+beginz(p),             1) : ...
               choose(endz(p)>=0, newmsize(dim(p))-endz(p),newmsize(dim(p)));
end

% do it
f = repmat(what,newmsize);
f(sf{:}) = m(sm{:});
