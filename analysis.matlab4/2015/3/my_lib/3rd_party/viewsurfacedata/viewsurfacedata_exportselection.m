function selection = viewsurfacedata_exportselection(snum)

% function selection = viewsurfacedata_exportselection(snum)
%
% <snum> (optional) is the number of the selection surface.
%   if [] or not supplied, defaults to the current selection surface.
%
% return the specified (or current) selection matrix.
% note that the form depends on the associated selection mode.

global VS_GUI;

% deal with input
if ~exist('snum','var') || isempty(snum)
  snum = [];
end

% init
selection = [];

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
  
% check more
if ~(all(isint(snum)) && snum>=1 && snum<=length(handles.ssurf))
  fprintf(1,'error: <snum> is not valid.\n');
  return;
end

% do it
sval = handles.ssurf(snum);
switch handles.surfacerecord{sval}.selectionmode
case 1
  selection = handles.epiclicked{sval};
  fprintf(1,'fyi, there are %d selected voxels out of %d total data voxels.\n', ...
          count(selection),numel(selection));
case 2
  selection = handles.vclicked{sval};
  fprintf(1,'fyi, there are %d selected vertices out of %d total vertices.\n', ...
          count(selection),length(selection));
end
