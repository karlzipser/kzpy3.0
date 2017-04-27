function I = viewsurfacedata_voxeltoindex(X,Y,Z)

% function I = viewsurfacedata_voxeltoindex(X,Y,Z)
%
% <X>,<Y>,<Z> are (unrectified) data subscripts, possibly with repeats
%
% return indices of (unrectified) data indices, with repeats removed.

global VS_R VS_RPTR;

% define
xyzsize = VS_R{VS_RPTR{1}}.xyzsize;

% do it
I = unique(sub2ind(xyzsize,X,Y,Z));  % remove repeats
