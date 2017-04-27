function viewsurfacedata_importcamerapreset(preset)

% function viewsurfacedata_importcamerapreset(preset)
%
% <preset> is a camera preset vector
%
% change the camera to <preset>.  the render window
% is immediately redrawn.

global VS_GUI;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% set it
handles.importcamerapreset = preset;

% force redraw (this saves implicitly i think)
viewsurfacedata_gui('redraw_Callback',handles.redraw,[],handles);

% report
fprintf(1,'camera preset successfully set.\n');
