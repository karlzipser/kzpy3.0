function viewsurfacedata_disablerender

% function viewsurfacedata_disablerender
%
% safely make render window impotent.

global VS_FIG;
if ishandle(VS_FIG)
  set(VS_FIG,'CloseRequestFcn','closereq');
  set(VS_FIG,'WindowButtonDownFcn','');
end
