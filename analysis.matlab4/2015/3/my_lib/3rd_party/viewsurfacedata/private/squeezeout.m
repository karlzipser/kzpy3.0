function f = squeezeout(m,dims)

% function f = squeezeout(m,dims)
%
% <m> is a matrix
% <dims> is a vector of dimension indices.  these should refer to 
%   singleton dimensions (we give error if this is not true).
%
% return the result of squeezing out the dimensions <dims>.

% check dims
for p=1:length(dims)
  assert(size(m,dims(p))==1,'dimension not singleton');
end

% calc
maxdim = max(ndims(m),max(dims));
msize = sizefull(m,maxdim);
msize(dims) = [];

% do it
f = reshape(m,msize);
