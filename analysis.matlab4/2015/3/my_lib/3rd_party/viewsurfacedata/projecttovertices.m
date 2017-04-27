function [f,dataindices,dataindicesL] = projecttovertices(m,srf,tr,xyzbegin,method,mode)

% function [f,dataindices,dataindicesL] = projecttovertices(m,srf,tr,xyzbegin,method,mode)
%
% <m> is a 4D data matrix (one or more 3D volumes)
% <srf> is
%   (1) the cell vector of surface parameters
%   (2) 4 x N coordinate matrix
% <tr> is the transformation (either a struct or a 4D matrix telling us
%    how to go from coordinate space to slice space).  can also be a cell
%    vector of <tr> things.
% <xyzbegin> (optional) is the matrix begin location.
%   if [] or not supplied, default to [1 1 1].
% <method> (optional) is the interpolation method:
%   'linear' | 'cubic' | 'nearest' | 'spline'
%   if [] or not supplied, default to 'linear'.
% <mode> (optional) is like in matrixinterpn:
%   0 means pad with a single layer of zeros around all edges
%   1 means pad by repeating edges to create an extra single layer
%     around all edges.
%   2 means don't pad.
%   if [] or not supplied, default to 0.
% 
% project the 3D slices of <m> onto <srf> according to <tr> and
% <xyzbegin>.  the interpolation scheme is given by <method> and <mode>.
%
% return the vertex data in <f> as T x V where
% T corresponds to the 3D slices of <m> and where
% V are vertices.
%
% return the mapping from vertices to voxels in <dataindices>.
% this is 1 x V, where V are vertices and where values
% are the index into a 3D slice of <m>.  (note that in the case that
% <tr> is a cell vector, this reflects only the last 3D volume slice.)
% also, return <dataindicesL>, a binary matrix indicating which voxels
% are involved in linear interpolation of voxel values onto the vertices.
%
% note that the 3D slices of <m> are completely independent
% and do not affect each other.
%
% note that the 3D slices of <m> can be complex-valued, in which
% case the real and imaginary parts are independently interpolated.

% deal with input
if ~exist('xyzbegin','var') || isempty(xyzbegin)
  xyzbegin = [1 1 1];
end
if ~exist('method','var') || isempty(method)
  method = 'linear';
end
if ~exist('mode','var') || isempty(mode)
  mode = 0;
end

% do it
  % load surface
if iscell(srf)
  XYZ = loadvtk(srf{:});
else
  XYZ = srf;
end
  % init
f = zeros([size(m,4) size(XYZ,2)]);
  % ok
fprintf(1,'working');
for p=1:size(m,4)
  if p==1 || iscell(tr)  % if not a cell, then only do it the first time
    if iscell(tr)
      tr0 = tr{p};
    else
      tr0 = tr;
    end
      % project to volume space (??)
    if isstruct(tr0)
      XYZ = volumetoslices(XYZ,tr0);
    else
      XYZ = tr0*XYZ;
    end
      % adjust for matrix space
    XYZ(1,:) = XYZ(1,:) - (xyzbegin(1)-1);
    XYZ(2,:) = XYZ(2,:) - (xyzbegin(2)-1);
    XYZ(3,:) = XYZ(3,:) - (xyzbegin(3)-1);
      % interpolate each time point
    temp = {XYZ(1,:) XYZ(2,:) XYZ(3,:)};
  end
  f(p,:) = matrixinterpn(m(:,:,:,p),temp,method,[],mode);
  fprintf(1,'.');
end
fprintf(1,'\n');

  % round and determine data indices
XYZr = round(XYZ);
dataindices = sub2ind2(sizefull(m(:,:,:,1),3),XYZr(1,:),XYZr(2,:),XYZr(3,:));

  % those involved in linear interpolation
xyzsize = sizefull(m(:,:,:,1),3);
dataindicesL = zeros(xyzsize);
for p=1:size(XYZ,2)
  dataindicesL(max(1,floor(XYZ(1,p))):min(xyzsize(1),ceil(XYZ(1,p))), ...
               max(1,floor(XYZ(2,p))):min(xyzsize(2),ceil(XYZ(2,p))), ...
               max(1,floor(XYZ(3,p))):min(xyzsize(3),ceil(XYZ(3,p)))) = 1;
end
