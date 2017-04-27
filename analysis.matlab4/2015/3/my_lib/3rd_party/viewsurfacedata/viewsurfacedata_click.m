function viewsurfacedata_click(pt)

% function viewsurfacedata_click(pt)
%
% <pt> is either a voxel like [35 15 16] or a vertex like 16536
%
% in the case of <pt> being a voxel (and voxel mode being available),
% this function simulates a click on some vertex within the voxel.  
% in the case of <pt> being a vertex, this function simulates a 
% click on that vertex.
%
% it is okay to call this function while in line, polygon, 
% or queue mode.

global VS_GUI VS_RVNUM VS_R VS_RPTR VS_TISOLATED;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% check
if ~(handles.epion && isrowvector(pt) && numel(pt)==3 && all(isint(pt)) && all(pt>=1) && all(pt<=VS_R{VS_RPTR{1}}.matrixsizereal) || ...
     isscalar(pt) && all(isint(pt)) && pt>=1 && pt<=VS_RVNUM)
  fprintf(1,'error: <pt> is not syntactically valid.\n');
  return;
end

% pull out info
[dindices,vindices] = viewsurfacedata_access(1);

% if voxel
if numel(pt)==3
  idx = viewsurfacedata_voxeltoindex(pt(1),pt(2),pt(3));
  fd = find(dindices==idx & ~VS_TISOLATED(vindices));
  if isempty(fd)
    fprintf(1,'error: there is no non-isolated vertex contained within the specified voxel.\n');
    return;
  else
    vertex = vindices(firstel(fd));
  end
% else, vertex
else
  if VS_TISOLATED(pt)
    fprintf(1,'error: the specified vertex is isolated and cannot be clicked on.\n');
    return;
  end
  vertex = pt;
end

% do it
viewsurfacedata_gui('selectionclick_Callback',0,[],handles,vertex);

% report
fprintf(1,'successfully performed a simulated click.\n');
