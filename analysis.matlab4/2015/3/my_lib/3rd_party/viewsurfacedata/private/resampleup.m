function f = resampleup(m,upsample,elt)

% function f = resampleup(m,upsample,elt)
% 
% <m> is a matrix, possibly empty
% <upsample> is a vector of positive real numbers which indicate
%   for each dimension, the number of elements in the resampled 
%   volume that should correspond to a single element in the 
%   original volume.  the length of <upsample> can be smaller or
%   larger than the number of dimensions of <m>.  (in the case
%   that the length of <upsample> is less than the number of
%   dimensions of <m>, ones are automatically assumed.)
% <elt> (optional) is the element to insert.  if [] or not supplied,
%   the behavior is to use nearest neighbor interpolation.  for example,
%   if <elt> is 0, then at a <upsample> factor of 4, 1 becomes 1 0 0 0.
%   note that the 1 is anchored at the "first" corner.
%
% return <m>, resampled up.

% NOTE: DEPRECATED IN FAVOR OF UPSAMPLEMATRIX.m.
% THE ONLY FUNCTIONALITY THAT UPSAMPLEMATRIX LACKS IS DECIMALS FOR <UPSAMPLE>

% deal with input
if ~exist('elt','var')
  elt = [];
end

% pad as necessary
numdims = max(ndims(m),length(upsample));
upsample = placematrix(ones(1,numdims),upsample,[1 1]);
newsize = sizefull(m,numdims).*upsample;

% get out early
if all(upsample==1)
  f = m;
  return;
end

% ok, if nearest neighbor
if isempty(elt)
  
  % determine indices
  idx = {};
  for p=1:numdims
    idx{p} = round(resamplingindices(1,size(m,p),-upsample(p)));
    warning('hrmm....   is the above line correct for the case of decimals?');
  end
  
  % do it
  f = m(idx{:});

% otherwise, use <elt>
else

  % init
  f = repmat(elt,newsize);
  
  % determine indices
  idx = {};
  for p=1:numdims
    idx{p} = linspacefixeddiff(1,upsample(p),size(m,p));
  end

  % fill
  f(idx{:}) = m;

end
