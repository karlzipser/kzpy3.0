function [neighbors,isolated] = facestoneighbors2(faces,numvertices)

% function [neighbors,isolated] = facestoneighbors2(faces,numvertices)
%
% <faces> is N x 3, where elements are the 1-indexed vertex indices
% <numvertices> is the number of vertices expected
%
% return <neighbors> as a matrix of size <numvertices> x N,
% where row i contains distinct neighbor vertices for vertex i,
% positioned left-justified with any remaining slots filled with 0.
% return <isolated> as a vector of size
% 1 x <numvertices> with elements 0/1 indicating
% whether that vertex is not involved in any face.
%
% note that the neighbor vertices of a vertex will be listed in
% an order reflecting successive connections.  it is required
% that the faces defined by <faces> all have the same ordering,
% e.g., CCW or CW.  moreover, it is required that every vertex 
% is completely surrounded by a series of adjacent triangles;
% another way to say this is that it is required that there are
% no holes in the surface (so isomorphic to a sphere, i think).

% init
neighbors = zeros(numvertices,0);
isolated = ones(1,numvertices);

% construct
bigone = sortrows([faces; circshift(faces,[0 1]); circshift(faces,[0 -1])]);
sz = size(bigone,1);

% do it
cnt = 1;
while cnt<=sz
  cntold = cnt;
  vidx = bigone(cnt,1);
  while cnt<=sz && bigone(cnt,1)==vidx
    cnt = cnt + 1;
  end
  vn = joinpairs(bigone(cntold:cnt-1,2:3));
  if size(neighbors,2) < length(vn)
    neighbors = placematrix(zeros([size(neighbors,1) length(vn)]),neighbors,[1 1]);
  end
  neighbors(vidx,1:length(vn)) = vn;
  isolated(vidx) = 0;
end
