function f = extractsubvolume(m,xyzbegin,xyzsize)

% function f = extractsubvolume(m,xyzbegin,xyzsize)
%
% <m> is a matrix
% <xyzbegin> is a position vector of length >= ndims(m)
% <xyzsize> is a matrix size
%
% return the subvolume positioned at <xyzbegin> with
% size <xyzsize>.

% sanity check
assert(length(xyzbegin) >= ndims(m));

% calc, adjust
numdims = max(ndims(m),length(xyzbegin));
xyzsize = placematrix(ones(1,numdims),xyzsize,[1 1]);

% do it
index = cell(1,numdims);
for p=1:numdims
  index{p} = (xyzbegin(p)-1) + (1:xyzsize(p));
end
f = subscript(m,index);
