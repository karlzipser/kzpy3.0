function f = viewsurfacedata_faceintersect(d)

% function f = viewsurfacedata_faceintersect(d)
%
% d is 1,2,3 which refers to the epi dimensions
%
% return M x 3 matrix.
%   M is the number of faces in the reference surface.
%   a row with all NaNs means that it doesn't intersect any plane
%   a row = [a b c] means a is the scalar from vertex 1 to 2
%                         b is the scalar from vertex 2 to 3
%                         c is the scalar from vertex 3 to 1
%     where intersections occur only for scalars in [0,1]
%       and where Inf means no intersection
%
% note that the planes are taken with respect to the
% transformation associated with the first surface.
% accordingly, the first surface had better be the
% voxel-specification case (else we die).

global VS_RFACES VS_RPTR VS_R VS_RXYZ;

idx = VS_RPTR{1};
assert(~isempty(idx));

% speed-up
mn = min(VS_R{idx}.raw(d,:));
mx = max(VS_R{idx}.raw(d,:));
pstart = max(0,ceil(mn-0.5));
pend = min(VS_R{idx}.matrixsizereal(d),floor(mx-0.5));

% do it
f = findfaceintersections(VS_R{idx}.raw,VS_RFACES,d,(pstart+.5):(pend+.5));

%  good1 = p1(1,:) >= .5 & p1(1,:) <= .5+VS_R{idx}.matrixsizereal(1) & ...
%          p1(2,:) >= .5 & p1(2,:) <= .5+VS_R{idx}.matrixsizereal(2) & ...
%          p1(3,:) >= .5 & p1(3,:) <= .5+VS_R{idx}.matrixsizereal(3);
%  good2 = p2(1,:) >= .5 & p2(1,:) <= .5+VS_R{idx}.matrixsizereal(1) & ...
%          p2(2,:) >= .5 & p2(2,:) <= .5+VS_R{idx}.matrixsizereal(2) & ...
%          p2(3,:) >= .5 & p2(3,:) <= .5+VS_R{idx}.matrixsizereal(3);
%  good3 = p3(1,:) >= .5 & p3(1,:) <= .5+VS_R{idx}.matrixsizereal(1) & ...
%          p3(2,:) >= .5 & p3(2,:) <= .5+VS_R{idx}.matrixsizereal(2) & ...
%          p3(3,:) >= .5 & p3(3,:) <= .5+VS_R{idx}.matrixsizereal(3);
%  good = good1 | good2 | good3;
%
%  s1 = ((.5+p) - p1(d,good))./(p2(d,good)-p1(d,good));  % 1 x F2, scalar interp from first to second
%  s2 = ((.5+p) - p2(d,good))./(p3(d,good)-p2(d,good));  % 1 x F2, scalar interp from second to third
%  s3 = ((.5+p) - p3(d,good))./(p1(d,good)-p3(d,good));  % 1 x F2, scalar interp from third to first
%
%  f(subscript(find(ifaces),good),:) = [s1; s2; s3]';
