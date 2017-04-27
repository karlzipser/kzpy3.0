function pobj = viewsurfacedata_drawboundaries(pobj,dim,offset)

% function pobj = viewsurfacedata_drawboundaries(pobj,dim,offset)
%
% pobj is an existing patch handle to mangle.
%   if [], then we make a new patch.
% dim is 1, 2, or 3
% offset is the real offset factor (already multipled by constant).
%
% draw and return a handle to a patch object.
% note that the conditions for the voxel boundaries case must
% already be satisfied (see sanity check below).

global VS_R VS_RPTR VS_TXYZ VS_TNORMALS;

% handle input
if isempty(pobj)
  pobj = patch;
end

% sanity check
assert(~isempty(VS_RPTR{1}));

% calcs
suffix = ['vbound',num2str(dim)];

% do we have the necessary information?
if ~isfield(VS_R{VS_RPTR{1}},suffix)
  fprintf(1,'calculating voxel boundaries for dimension %d...please wait...\n',dim);
  VS_R{VS_RPTR{1}}.(suffix) = viewsurfacedata_vbounds(dim);
  fprintf(1,'done!\n');
end

% ok...
[faces,vertices,fvad] = joinfaceintersections(VS_TXYZ,VS_R{VS_RPTR{1}}.(suffix),VS_TNORMALS,offset);

% draw it! (the master version is in _drawsurface; keep me up-to-date!)
set(pobj,    'Faces',faces,                    ...
             'Vertices',vertices,              ...
             'NormalMode','manual',            ...  % leave manual, but note that we don't use normals!
             'FaceVertexAlphaData',fvad,       ...
             'AlphaDataMapping','none',        ...
             'BackFaceLighting','unlit',       ...
             'EdgeLighting','none',            ...
             'MarkerSize',6,                   ...
             'MarkerEdgeColor','white',        ...
             'FaceAlpha',0,                    ...  % invariant
             'FaceColor','none',               ...  % invariant
             'LineWidth',1,                    ...
             'Clipping','on');
