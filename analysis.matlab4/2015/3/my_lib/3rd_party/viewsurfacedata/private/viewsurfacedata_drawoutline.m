function pobj = viewsurfacedata_drawoutline(pobj,vindices)

% function pobj = viewsurfacedata_drawoutline(pobj,vindices)
%
% <pobj> is an existing patch handle to mangle.
%   if [], then we make a new patch.
% <vindices> is a vector of vertex indices, which should be
%   already be free of isolated vertices
%
% draw and return a handle to a patch object.

global VS_TXYZ VS_TNORMALS;

% handle input
if isempty(pobj)
  pobj = patch;
end

% ok, get the info
[connections,edgevindices] = viewsurfacedata_outline(vindices);

% ok, try to connect edges up (TODO: consider doing in perl?)
chains = {};
[rows,cols] = find(connections);
while ~isempty(rows)
  % seed
  cur = rows(1);
  record = [cur];
  % start loop
  temp = find(rows==cur);
  next = firstel(cols(temp));
  while ~isempty(next)
    record = [record next];
    rows(temp(1)) = [];
    cols(temp(1)) = [];
    cur = next;
    temp = find(rows==cur);
    next = firstel(cols(temp));
  end
  % record
  chains{end+1} = record;
end

% ok, now every found edge is involved in some chain.

% init
faces = [];
vertices = [];
vertexnormals = [];
fvad = [];
vertexcnt = 0;

% do it (mirrored in joinfaceintersections!)
for p=1:length(chains)
  chain = chains{p};
  % check for circular case
  if chain(1)==chain(end)
    iscircular = 1;
    chain(end) = [];
  else
    iscircular = 0;
  end
  % calc
  numvertices = length(chain);
  % define vertices + vertexnormals
  vertices = [vertices; VS_TXYZ(1:3,chain)'];
  vertexnormals = [vertexnormals; VS_TNORMALS(1:3,chain)'];
  % define faces; we repeat the last vertex (so like [1 2 3 3 3 3 3 ...].  FIXME: it is a slowdown?
  if ~isempty(faces)
    if size(faces,2) < numvertices  % enlarge faces if necessary
      faces = [faces repmat(faces(:,end),[1 numvertices-size(faces,2)])];
    end
    newface = placematrix(repmat(vertexcnt+numvertices,[1 size(faces,2)]),vertexcnt+1:vertexcnt+numvertices,[]);
%OLD:
%     newface = matrixpad2(vertexcnt+1:vertexcnt+numvertices, ...
%                          [1 size(faces,2)],'center',vertexcnt+numvertices);  % FIXME: shouldn't this be 'first'????
% FIXME: IS THE CENTERING RIGHT IN PLACEMATRIX CALL ABOVE?
  else
    newface = vertexcnt+1:vertexcnt+numvertices;
  end
  faces = [faces; newface];
  % define fvad
  newfvad = ones(numvertices,1);  % numvertices x 1
  if ~iscircular
    % the last edge is the one that feeds into the first vertex.
    % so set the alpha of the first vertex to 0.
    % the MATLAB documentation is wrong!!
    newfvad(1) = 0;
  end
  fvad = [fvad; newfvad];
  % increment
  vertexcnt = vertexcnt+numvertices;
end

% well, what we have done is to define a bunch of chains of edges.
% we still care about those set vertices that did not make it into an edge.
% so we include fake degenerate faces for these, just so they show up
% when we turn on markers.
outliers = setdiff(vindices,edgevindices);
faces = [faces; repmat((vertexcnt+1:vertexcnt+length(outliers))',[1 size(faces,2)])];
vertices = [vertices; VS_TXYZ(1:3,outliers)'];
vertexnormals = [vertexnormals; VS_TNORMALS(1:3,outliers)'];
fvad = [fvad; ones(length(outliers),1)];

% draw it! (the master version is in _drawsurface; keep me up-to-date!)
set(pobj,    'Faces',faces,                    ...
             'Vertices',vertices,              ...
             'VertexNormals',vertexnormals,    ...
             'NormalMode','manual',            ...
             'FaceVertexCData',[0 0 0],        ...  % bogus value, but need to stuff something in
             'FaceVertexAlphaData',fvad,       ...
             'AlphaDataMapping','none',        ...
             'BackFaceLighting','unlit',       ...
             'EdgeLighting','none',            ...
             'MarkerSize',6,                   ...
             'MarkerEdgeColor','white',        ...
             'FaceAlpha',0,                    ...  % invariant
             'FaceColor','none',               ...  % invariant
             'LineWidth',2,                    ...
             'Clipping','on');

% set additional stuff
set(pobj,'UserData',struct('origvertices',vertices));  % not saving vindices (cf _drawsurface) since never used.
