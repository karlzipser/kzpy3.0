function [neighbors,isolated] = facestoneighbors(faces,numvertices)

% function [neighbors,isolated] = facestoneighbors(faces,numvertices)
%
% <faces> is N x 3, where elements are the 1-indexed vertex indices
% <numvertices> is the number of vertices expected
%
% return <neighbors> as a cell vector of size 1 x <numvertices>
% where each element is a matrix, each column
% of which is a pair of vertex indices and corresponds
% to a face involving the vertex associated with that
% matrix; each matrix should include all faces involving
% the associated vertex.  return <isolated> as a vector of size
% 1 x <numvertices> with elements 0/1 indicating whether that
% vertex is not involved in any face.
%
% TODO: this function is really slow

% init
neighbors = cell(1,numvertices);
isolated = ones(1,numvertices);

% construct
bigone = sortrows([faces; circshift(faces,[0 1]); circshift(faces,[0 -1])]);
sz = size(bigone,1);

% do it
cnt = 1;
while cnt<=sz
%  if mod(cnt,1000)==0
%    fprintf(1,'%d/%d,',cnt,sz);
%  end
  cntold = cnt;
  vidx = bigone(cnt,1);
  while cnt<=sz && bigone(cnt,1)==vidx
    cnt = cnt + 1;
  end
  neighbors{vidx} = bigone(cntold:cnt-1,2:3)';
  isolated(vidx) = 0;
end
%fprintf(1,'\n');
