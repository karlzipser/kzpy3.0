function viewsurfacedata_autosnapshot2

% function viewsurfacedata_autosnapshot(surfaces)
%
% <surfaces> (optional) is a vector of surface indices.  if []
%   or not supplied, default to indices for regular surfaces.
%
% for each surface S listed in <surfaces>, set all regular surfaces
% to be not visible, set surface S to be visible, redraw, and 
% take a snapshot.  after finished, return to the original 
% visibility settings.

global VS_GUI;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% do it
writesnapshot(handles.fig,[],[],[],[],handles.settings.snapshotmethod);  % mirrors snapshot_Callback

% report
fprintf(1,'auto-snapshot is complete!\n');
