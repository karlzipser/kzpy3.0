function vindicesconnect = viewsurfacedata_connectpts(vindices)

% function vindicesconnect = viewsurfacedata_connectpts(vindices)
%
% <vindices> is a vector of vertex indices.  (this better not
%   include any isolated vertices, and shouldn't since it 
%   should be impossible to select an isolated vertex.)
%
% for each successive pair of vertices specified in <vindices>,
% we work our way across the target surface, minimizing Euclidean
% distance at each step.  we return <vindicesconnect>, a vector
% of vertex indices indicating (in order) the vertices that we
% visited, including the beginning and ending vertices.
%
% this function does assume that there exists a path 
% (with respect to the target surface) connecting the vertex
% indices you pass in.
%
% if you have two successive vertex indices that are the same,
% we don't repeat that vertex index in <vindicesconnect>.

global VS_TXYZ VS_TNEIGHBORS;

% deal with degenerate case
if isempty(vindices)
  vindicesconnect = [];
  return;
end

% seed
vindicesconnect = [vindices(1)];
cur = vindices(1);

% start loop
cnt = 2;
while cnt<=length(vindices)
  target = vindices(cnt);
  targetc = VS_TXYZ(1:3,target);
  while cur~=target
    potential = unique(flatten(VS_TNEIGHBORS{cur}));
    potentialc = VS_TXYZ(1:3,potential);
    [dummy,idx] = min(sum((repmat(targetc,[1 length(potential)])-potentialc).^2,1));
    next = potential(idx);
    vindicesconnect = [vindicesconnect next];
    cur = next;
  end
  cnt = cnt + 1;
end
