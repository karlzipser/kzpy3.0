function viewsurfacedata_savesettings(x)

% function viewsurfacedata_savesettings(x)
%
% given settings in struct <x>, save settings via setpref.m.
% struct <x> need not be a complete set of settings.
% any existing settings are overwritten on a per-setting basis.

xorig = getpref('kendrick','viewsurfacedata',struct([]));
x = mergestructs(x,xorig);
setpref('kendrick','viewsurfacedata',x);


% OLD
% file = viewsurfacedata_constants('settingsfile');
% savesettings(x,file);
