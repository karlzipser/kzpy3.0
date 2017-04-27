function [m,posrecord] = matrixpad2(m,dsize,pos,what)

% function [m,posrecord] = matrixpad2(m,dsize,pos,what)
%
% <m> is a matrix, possibly empty
% <dsize> is the size we want it to be (can be bigger or smaller or the same)
% <pos> is a vector of same size as <dsize>, specifying the position offset for
%   the corresponding dimension, where elements are integers or 'center' or
%   'first' or 'last'.  (thus, <pos> can be a cell vector.)  'center' means 
%   to center within that dimension.  'first' means 1.  'last' means the 
%   last position which does not cause overage.  also, <pos> can be a
%   single element, in which case we use that value for all the dimensions.
%   also, <pos> can be [], in which case this means the same thing as 'center'.
% <what> (optional) is the element that we use for padding (if we need it).
%   if [] or not supplied, default to 0.
%
% <m> is returned as the padded matrix.
% <posrecord> stores the actual position offsets we used (like
%   <pos> but can't be [] and can't have -1).  to be explicit, 
%   posrecord(x) is the position of the first element in
%   dimension x.
%
% this routine is potentially slow.

% deal with input
if ~exist('what','var') || isempty(what)
  what = 0;
end
if isempty(pos)  % default
  pos = 'center';
end
if ischar(pos)  % convert to cell
  pos = {pos};
end
if ~iscell(pos)  % convert to cell
  pos = num2cell(pos);
end
if length(pos)==1  % repeat
  pos = repmat(pos,1,length(dsize));
end

posrecord = [];
dim = [];
beginz = [];
endz = [];

if ~isempty(m)
  msize = ones(1,max([ndims(m) length(dsize)]));
  msize(1:ndims(m)) = size(m);
else
  msize = zeros(1,length(dsize));  % careful!
end
for p=1:length(dsize)
  temp = dsize(p) - msize(p);
  switch pos{p}
  case 'center'
    posrecord(p) = round(temp/2)+1;
  case 'last'
    posrecord(p) = temp+1;
  case 'first'
    posrecord(p) = 1;
  otherwise
    posrecord(p) = pos{p};
  end
  if posrecord(p)-1~=0 | temp-(posrecord(p)-1)~=0
    dim = [ dim p ];
    beginz = [ beginz posrecord(p)-1 ];
    endz = [ endz temp-(posrecord(p)-1) ];
  end
end
if ~isempty(dim)
  m = matrixpad(m,dim,beginz,endz,what);
end
