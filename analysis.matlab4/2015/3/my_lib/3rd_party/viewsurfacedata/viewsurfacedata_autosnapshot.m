function viewsurfacedata_autosnapshot(surfaces)

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

% save initial visibility setting
orig = viewsurfacedata_toggle([],0);

% deal with input
if ~exist('surfaces','var') || isempty(surfaces)
  surfaces = handles.rsurf;
end

% check input
if ~(isrowvector(surfaces) && all(isint(surfaces) & surfaces>=1 & surfaces<=length(orig)))
  fprintf(1,'error: invalid format for <surfaces>.\n');
  return;
end
  
% do it
for p=1:length(surfaces)
  visibility = orig;
  visibility(handles.rsurf) = 0;
  visibility(surfaces(p)) = 1;
  fprintf(1,'working on surface index %d.\n',surfaces(p));
  viewsurfacedata_toggle(visibility,0);
  writesnapshot(handles.fig,[],[],[],[],handles.settings.snapshotmethod);  % mirrors snapshot_Callback
end  

% restore original settings
viewsurfacedata_toggle(orig,0);

% report
fprintf(1,'auto-snapshot is complete!\n');
