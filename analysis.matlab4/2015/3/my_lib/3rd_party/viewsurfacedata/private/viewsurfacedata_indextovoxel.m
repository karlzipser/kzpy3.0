function [X,Y,Z] = viewsurfacedata_indextovoxel(dindices)

% function [X,Y,Z] = viewsurfacedata_indextovoxel(dindices)
%
% <dindices> is a vector of (unrectified) data indices.
%
% return subscripts of (unrectified) data indices.

global VS_R VS_RPTR;

% define
xyzsize = VS_R{VS_RPTR{1}}.xyzsize;

% do it
[X,Y,Z] = ind2sub(xyzsize,dindices);
