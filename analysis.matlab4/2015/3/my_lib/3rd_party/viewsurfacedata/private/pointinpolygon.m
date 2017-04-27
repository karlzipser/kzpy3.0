function inout = pointinpolygon(polygon)

% function inout = pointinpolygon(polygon)
%
% polygon: vertices of the polygon, M x 2
%
% globals:
%   PIP_VERTICES: N (vertices) x 2 (coordinates per vertex)
%
% return inout, which is a vector of length
% size(PIP_VERTICES,1) where values indicate whether
% the vertex is inside the polygon.
%
% note that this function is in two-dimensions.
%
% NOTE: this is based on code contained in pointinpolygon.cpp!

% globals
global PIP_VERTICES;

wn = 0;
for p=1:size(polygon,1)
  % define
  pt1 = polygon(p,:);
  pt2 = polygon(mod2(p+1,size(polygon,1)),:);
  % define more
  test1 = pt1(2) <= PIP_VERTICES(:,2);
  test2 = pt2(2) > PIP_VERTICES(:,2);
  il = isleft(pt1,pt2,PIP_VERTICES);
  % increments
  wn = wn + (test1 & test2 & (il>0));
  wn = wn - (~test1 & ~test2 & (il<0));
end

inout = wn~=0;  % anything non-zero is inside!
 
%%%%%%%%%%%%%%%%%%%

function f = isleft(pt1,pt2,points)
f = (pt2(1)-pt1(1)) * (points(:,2)-pt1(2)) - (points(:,1)-pt1(1)) * (pt2(2)-pt1(2));
