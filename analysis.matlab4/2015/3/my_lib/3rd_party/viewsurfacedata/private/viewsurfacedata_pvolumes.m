function [pvv,pvv1,pvv2,pvvvoxel] = viewsurfacedata_pvolumes

% function [pvv,pvv1,pvv2,pvvvoxel] = viewsurfacedata_pvolumes
%
% return the following variables:
%
% <pvv>:
% matrix with same dimensions as data where elements
% are the number of within-surfaces for that voxel.
% for example, a normal (non-partial-volume) voxel
% gets a value of 1.
%
% <pvv1>:
% [ A1 A2 B1 C1 ... ]
%
% <pvv2>:
% [ A2 A3 B2 C2 ... ]
%
% <pvvvoxel>:
% [ A  A  B  C ... ]
%
% in the above example, voxel A has three within-surfaces.
% voxels B and C have two surfaces each.  the entries in
% <pvv1> and <pvv2> are vertex indices (specifically, each
% vertex index is the closest vertex to the average
% coordinate of all the vertices in a single within-surface 
% (in the reference surface)).  lines are to be drawn
% between corresponding entries of <pvv1> and <pvv2>.  note that
% if there are N within-surfaces, there will be N-1 lines 
% to draw (namely, lines between consecutive pairs of
% within surfaces).  <pvvvoxel> has the same dimensions
% as <pvv1> and <pvv2> and indicates the voxel index 
% associated with each line.

global VS_R VS_RPTR VS_RNEIGHBORS VS_RXYZ VS_RVNUM VS_RXYZ;

% sanity check
assert(~isempty(VS_RPTR{1}));

% define
valid = VS_R{VS_RPTR{1}}.valid;      % vertex indices inside voxels
indices = VS_R{VS_RPTR{1}}.indices;  % the associated voxel indices
processed = zeros(VS_R{VS_RPTR{1}}.xyzsize);
pvv = ones(VS_R{VS_RPTR{1}}.xyzsize);  % default value is 1
pvv1 = [];
pvv2 = [];
pvvvoxel = [];

% calculate
  % this is for all vertices.
  % NaN indicates no voxel mapping; a number indicates the index of the voxel mapped to.
indices2 = NaN*zeros(1,VS_RVNUM);
indices2(valid) = indices;

% for each vertex
for p=1:length(valid)
  % what voxel does it lie in?
  didx = indices(p);
  % if we haven't already processed this voxel
  if ~processed(didx)
    % update
    processed(didx) = 1;
    % what is the vertex index?
    vidx = valid(p);
    % what are the vertices within this voxel?
    allv = valid(indices==didx);
    % ok, process all of these vertices
    components = [];
    while ~isempty(allv)
      % determine a connected set of vertices and store in did
      grow = [allv(1)];  % seed
      did = [];
      while ~isempty(grow)
        root = grow(1);
        neighbors = flatten(VS_RNEIGHBORS{root});
        neighbors = neighbors(indices2(neighbors)==didx);  % filter for those in same voxel
        neighbors = setdiff(neighbors,[did grow]);  % filter for those not already done and not already queued up
        did = [did root];
        grow = [grow(2:end) neighbors];
      end
      % average and find closest
      avgpt = sum(VS_RXYZ(1:3,did),2)/length(did);
      [dummy,mnidx] = min(sum((VS_RXYZ(1:3,did) - repmat(avgpt,[1 length(did)])).^2,1));
      closest = did(mnidx);
      % update
      components = [components closest];
      allv = setdiff(allv,did);
    end
    % if there was more than one set of vertices
    if length(components) > 1
      pvv(didx) = length(components);
      for q=1:length(components)-1
        pvv1 = [pvv1 components(q)];
        pvv2 = [pvv2 components(q+1)];
        pvvvoxel = [pvvvoxel didx];
      end
    end
  end
end
