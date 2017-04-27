function f = viewsurfacedata_vbounds(dim)

% function f = viewsurfacedata_vbounds(dim)
%
% dim is 1, 2, or 3
%
% return the result from findfaceintersections.m.
%
% note that the planes are taken with respect to the
% transformation associated with the first surface.
% accordingly, the first surface had better be the
% voxel-specification case (else we die).

global VS_RFACES VS_RPTR VS_R VS_TNEIGHBORS;

idx = VS_RPTR{1};
assert(~isempty(idx));

% speed-up
mn = min(VS_R{idx}.raw(dim,:));
mx = max(VS_R{idx}.raw(dim,:));
pstart = max(0,ceil(mn-0.5));
pend = min(VS_R{idx}.matrixsizereal(dim),floor(mx-0.5));

% do it
limits = [.5 .5+VS_R{idx}.matrixsizereal(1);
          .5 .5+VS_R{idx}.matrixsizereal(2);
          .5 .5+VS_R{idx}.matrixsizereal(3)];
f = findfaceintersections(VS_R{idx}.raw,VS_RFACES,dim,(pstart+.5):(pend+.5),VS_TNEIGHBORS,limits);
