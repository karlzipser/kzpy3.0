function viewsurfacedata_redraw

% function viewsurfacedata_redraw
%
% redraw the render window.

global VS_GUI;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% force redraw
viewsurfacedata_gui('redraw_Callback',handles.redraw,[],handles);
