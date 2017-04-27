function [cvertex,dvertex] = verticestovoxels(vertices,tr,xyzbegin,xyzsize)

% function [cvertex,dvertex] = verticestovoxels(vertices,tr,xyzbegin,xyzsize)
%
% <vertices> is a set of points, 4 x N,
%   or a cell vector of such sets of points
% <tr> is of the format described in maketransformation.m
% <xyzbegin> is the location of the first voxel
% <xyzsize> is the effective matrix size
%
% return <cvertex> which is the index of the closest vertex,
% and <dvertex> which is the distance in mm to this closest vertex.
%
% we simply concatenate multiple sets of points together, and
% the vertex indices reflect this fact.

% deal with input
if ~iscell(vertices)
  vertices = {vertices};
end

% calc
len = tr.matrixfov./tr.matrixsize;

% combine vertices
vertices = cat(2,vertices{:});

% project to voxel space
vertices = volumetoslices(vertices,tr);

% adjust for xyzbegin
vertices(1,:) = vertices(1,:) - xyzbegin(1) + 1;
vertices(2,:) = vertices(2,:) - xyzbegin(2) + 1;
vertices(3,:) = vertices(3,:) - xyzbegin(3) + 1;

% do it
cvertex = zeros(xyzsize);
dvertex = zeros(xyzsize);
fprintf('working');
for p=1:xyzsize(1)  % could be made faster?
  fprintf('.');
  for q=1:xyzsize(2)
    for r=1:xyzsize(3)
      [dvertex(p,q,r),cvertex(p,q,r)] = ...
        min(len(1)^2 * (vertices(1,:) - p).^2 + len(2)^2 * (vertices(2,:) - q).^2 + len(3)^2 * (vertices(3,:) - r).^2);
    end
  end
end
fprintf('done.\n');

% adjust distance
dvertex = sqrt(dvertex);
