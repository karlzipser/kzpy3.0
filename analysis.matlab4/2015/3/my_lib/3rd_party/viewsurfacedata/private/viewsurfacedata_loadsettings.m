function f = viewsurfacedata_loadsettings(x,usedefaults)

% function f = viewsurfacedata_loadsettings(x,usedefaults)
%
% <x> is a cell vector of strings
% <usedefaults> is whether to completely ignore
%   existing values for the fields named in <x>.
%
% return a struct with the settings.

% define the default
inits = {'camerapresets'      {};
         'colormaps'          {};
         'viewangle'          15;
         'projection'         'perspective';
         'xdir'               'normal';
         'lighting'           'flat';
         'lightpos'           'headlight';
         'background'         'black';
         'edges'              'off';
         'markers'            'off';
         'visibility'         'on';
         'braincolor'         'white';
         'surfacecolor'       'hot';
         'selectioncolor'     'green';
         'curvaturecolor'     'black';
         'partialvolumecolor' 'cyan';
         'voxelboundcolor'    'black';
         'offset'             'zeros';
         'pvaluedir'          '<=';
         'pvalue'             1;
         'boundary'           'expand';
         'redrawmode'         'manual';
         'snapshotmethod'     'getframe'};
def = cell2struct(inits(:,2),inits(:,1),1);

% load in if possible
f = getpref('kendrick','viewsurfacedata',def);

% override certain ones
if usedefaults
  for p=1:length(x)
    f.(x{p}) = def.(x{p});
  end
end





% OLD
% inits = {'camerapresets'      {};
%          'colormaps'          {};
%          'viewangle'          15;
%          'projection'         'perspective';
%          'xdir'               'normal';
%          'lighting'           'flat';
%          'lightpos'           'headlight';
%          'background'         'black';
%          'edges'              'off';
%          'markers'            'off';
%          'visibility'         'on';
%          'braincolor'         'white';
%          'surfacecolor'       'hot';
%          'selectioncolor'     'green';
%          'curvaturecolor'     'black';
%          'partialvolumecolor' 'cyan';
%          'voxelboundcolor'    'black';
%          'offset'             'zeros';
%          'pvaluedir'          '<=';
%          'pvalue'             1;
%          'boundary'           'expand';
%          'redrawmode'         'manual';
%          'snapshotmethod'     'getframe'};
% file = viewsurfacedata_constants('settingsfile');
% f = loadsettings(x,usedefaults,inits,file);
