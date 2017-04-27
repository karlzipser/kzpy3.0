function viewsurfacedata_importselection(selection,snum)

% function viewsurfacedata_importselection(selection,snum)
%
% <selection> is one of the following:
%   (1) a matrix of the same dimensions as the first data matrix,
%       where 1 indicates that a voxel is "in" and 0 indicates that
%       a voxel is "out".  (note that voxels that do not intersect
%       the brain surface can still be specified to be "in", though
%       doing so is probably not useful.)
%   (2) a cell vector like {[x1 y1 z1] [x2 y2 z2] ...} where
%       elements refer to voxels that are to be in the selection
%       matrix.  this format is equivalent in power to (1) but
%       is included for convenience.
%   (3) a vector with length equal to the number of vertices in 
%       the reference/target surface, where 1 indicates that a vertex
%       is "in" and 0 indicates that a voxel is "out".
% <snum> (optional) is the number of the selection surface.
%   if [] or not supplied, defaults to the current selection surface.
%   if -1, import into a new selection surface.
%
% this function sets the specified (or current) selection
% surface according to <selection>, and sets the selection mode
% accordingly (cases (1) and (2) result in the 'voxel' mode;
% case (3) results in the 'vertex' mode).  the render window
% is then immediately redrawn.
%
% note that <snum> can be larger than the current number of
% selection surfaces, in which case new selection surfaces
% are automatically made for you.

global VS_GUI VS_RVNUM;

% deal with input
if ~exist('snum','var') || isempty(snum)
  snum = [];
end

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);
if isempty(snum)
  snum = get(handles.selectionsurface,'Value');
end
if snum==-1
  snum = length(handles.ssurf)+1;
end

% check
if ~(all(isint(snum)) && snum>=1)
  fprintf(1,'error: <snum> is not valid.\n');
  return;
end

% add new surfaces if needed (TODO: this should really be done only after input checks)
if snum > length(handles.ssurf)
  fprintf(1,'note that new selection surfaces are being added to accommodate request.\n');
  for p=1:snum-length(handles.ssurf)
    viewsurfacedata_gui('selectionsurfaceadd_Callback',handles.selectionsurfaceadd,[],handles);
    handles = guidata(VS_GUI);
  end
end

% deal with input
sval = handles.ssurf(snum);
if iscell(selection)
  if ~handles.epion
    fprintf(1,'error: specifying a cell vector is invalid since ''voxel'' mode is unavailable.\n');
    return;
  end
  handles.epiclicked{sval} = zeros(handles.dsize);
  for p=1:length(selection)
    handles.epiclicked{sval}(selection{p}(1),selection{p}(2),selection{p}(3)) = 1;  % FIXME: need more error-checking...
  end
  handles.surfacerecord{sval}.selectionmode = 1;
else
  if ~all(selection == 0 | selection == 1)
    fprintf(1,'error: specified selection matrix contains an element that is not 0 nor 1.\n');
    return;
  end
  if isrowvector(selection)
    if length(selection)~=VS_RVNUM
      fprintf(1,'error: specified selection matrix is a vector representing an incorrect number of vertices.\n');
      return;
    end
    handles.vclicked{sval} = selection;
    handles.surfacerecord{sval}.selectionmode = 2;
  else
    if ~handles.epion
      fprintf(1,'error: specifying a matrix is invalid since ''voxel'' mode is unavailable.\n');
      return;
    end
    if ~isequal(size(selection),handles.dsize)
      fprintf(1,'error: specified selection matrix is a matrix with incorrect dimensions.\n');
      return;
    end
    handles.epiclicked{sval} = selection;
    handles.surfacerecord{sval}.selectionmode = 1;
  end
end
handles.clickedchanged{sval} = 1;
handles.draworderchanged = 1;

% update the selection surface dependencies (this should not save handles, which
% is good since we call redraw_Callback next)
viewsurfacedata_gui('selectionsurface_Callback',handles.selectionsurface,[],handles);

% force redraw
viewsurfacedata_gui('redraw_Callback',handles.redraw,[],handles);

% report
fprintf(1,'selection successfully imported.\n');
