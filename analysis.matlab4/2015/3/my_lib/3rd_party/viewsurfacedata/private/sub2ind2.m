function f = sub2ind2(varargin)

% function f = sub2ind2(siz,i1,i2,...,in)
% 
% <siz> is the matrix size
% <i1>...<in> are the indices.  there must be at least one.
%
% like sub2ind, but allow for out of range indices (integral
% but can be < 1 or > maximum).

% calc
siz = varargin{1};
indices = varargin(2:end);

% determine bad
bad = {};
for p=1:length(indices)
  bad{p} = indices{p} < 1 | indices{p} > siz(p);
end
overallbad = repeatfunc(@or,bad{:});  

% fake the bad ones
for p=1:length(indices)
  indices{p}(overallbad) = 1;
end

% do it
f = sub2ind(siz,indices{:});
f(overallbad) = NaN;
