function [x,y,z] = getslicecoordinates(tr,matrixsize)

% function [x,y,z] = getslicecoordinates(tr,matrixsize)
%
% <tr> is:
%   (1) a transformation struct
%   (2) a 4x4 transformation matrix telling us how to go from
%       slice space to coordinate space
% <matrixsize> (optional) is the slice matrix size.
%   this is needed if and only if <tr> is of type (2)
%   since the slice matrix size is defined if <tr>
%   is of type (1).  if <tr> is of type (1) and <matrixsize>
%   is provided, it must match the (final) matrixsize of <tr>.
%
% here is the idea.  <tr> defines how to orient slices
% in coordinate space.  we return the x-, y-, and
% z-coordinates of the oriented slices.  the 
% output matrices <x>, <y>, and <z> have the
% same matrix dimension as the slices.

% VERIFY TRANSFORMATION struct?

% extract
if isstruct(tr)
  if isfield(tr,'extra')  % DEPRECATED
    ms = tr.extra.matrixsize;
    mf = tr.extra.matrixfov;
  else
    ms = tr.matrixsize;
    mf = tr.matrixfov;
  end
end

% check input
if exist('matrixsize','var')
  if isstruct(tr)
    assert(isequal(matrixsize,ms),'<matrixsize> does not match <tr>');
  end
else
  if ~isstruct(tr)
    error('since <tr> is a 4x4 transformation matrix, we require the <matrixsize> input');
  end
end

% define matrixsize for real if necessary
if ~exist('matrixsize','var')
  matrixsize = ms;
end

% get coordinates in coordinate space for the transformation
  % missing the speed-ups here...
[x,y,z] = reslicevolume(2,tr,'',3,[],1,0,[],[],[],[],[],matrixsize);
