function [connections,edgevindices] = viewsurfacedata_outline(vindices)

% function [connections,edgevindices] = viewsurfacedata_outline(vindices)
%
% <vindices> is a vector of vertex indices.  these can include isolated vertices,
%   which simply fall through and fail to show up in the outline, i think.
%
% return <connections>, a sparse connections matrix.
% return <edgevindices>, a vector of vertex indices that are involved in some edge.

global VS_TNEIGHBORS VS_TFACES VS_RVNUM VS_OUTLINEC VS_OUTLINEE VS_OUTLINEV;

% primitive caching
if isequal(vindices,VS_OUTLINEV)
  connections = VS_OUTLINEC;
  edgevindices = VS_OUTLINEE;
  return;
end

% internal constants
maxconnections = round(VS_RVNUM/100);  % how many edges to expect (FIXME?)

% hash to see which vertices are in our set
index = zeros(1,VS_RVNUM);
index(vindices) = 1;

% hash to see which set vertices are completely covered by other set vertices
covered = zeros(1,VS_RVNUM);
for p=vindices
  neighbors = VS_TNEIGHBORS{p};
  if all(index(neighbors))
    covered(p) = 1;
  end
end

% we need to determine all edges that connect two set vertices.
% the idea here is that any such edge will be part of some face,
% so let's find all faces involving at least two set vertices.
facesinvolve = sum(ismember(VS_TFACES,vindices),2);
facestwo = find(facesinvolve>=2);
facestwoexpand = VS_TFACES(facestwo,:);

% now define potential edges (2 x N).
% some of these edges may connect two vertices that aren't both
% set vertices; we'll deal with that later.
edges = [facestwoexpand(:,[1 2])' facestwoexpand(:,[1 3])' facestwoexpand(:,[2 3])'];

% any edge involving a covered vertex is definitely colored
% on both sides.  so we can exclude all edges involving a
% covered vertex.
edges(:,covered(edges(1,:)) | covered(edges(2,:))) = [];

% ok, edges that are colored on both sides are edges that
% are involved in two faces, both of which involve three
% vertices all of which are set vertices.  so we want
% to know which faces involve three set vertices.
facesthree = find(facesinvolve==3);
facesthreeexpand = VS_TFACES(facesthree,:);

% from the remaining edges, we want to find those that connect
% two set vertices and that aren't colored on both sides.  
% then for these found edges, we set them in a
% sparse connections matrix.  we also keep track of which
% vertices are involved in some edge.
connections = spalloc(VS_RVNUM,VS_RVNUM,maxconnections);
edgevindices = zeros(1,VS_RVNUM);
for edge=edges
  % if the edge connects two set vertices and if the number
  % of faces (of solely set vertices) involving these two
  % set vertices is not two...
  if all(index(edge)) && length(find(sum(facesthreeexpand==edge(1) | facesthreeexpand==edge(2),2)==2)) ~= 2
    connections(edge(1),edge(2)) = 1;
    connections(edge(2),edge(1)) = 1;
    edgevindices(edge) = 1;
  end
end
edgevindices = find(edgevindices);

% primitive caching
VS_OUTLINEV = vindices;
VS_OUTLINEC = connections;
VS_OUTLINEE = edgevindices;
