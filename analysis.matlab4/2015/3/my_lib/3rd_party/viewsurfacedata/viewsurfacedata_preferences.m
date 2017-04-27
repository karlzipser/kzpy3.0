function f = viewsurfacedata_preferences(prefs)

% function f = viewsurfacedata_preferences(prefs)
%
% f = viewsurfacedata_preferences
%   returns the existing saved preferences in struct f.
%
% f = viewsurfacedata_preferences('default')
%   returns the default preferences.
%
% viewsurfacedata_preferences(prefs)
%   given struct <prefs>, save the preferences.
%   <prefs> needs to to have all fields defined
%   (i.e. cannot be an incomplete struct).
%
%   here are the fields:
%
%   <colormaps> determines user-defined colormaps.
%     this is a cell vector of strings, each of which refers 
%     to a colormap-generating function, similar to functions
%     built into MATLAB (e.g., 'gray', 'hot', 'hsv').  see
%     viewsurfacedata.txt for more information.
%
%   <viewangle> determines the initial view angle setting
%   in degrees.
%     15 (default) | number in the range (0,Inf]
%
%   <projection> determines the initial projection setting.
%     'perspective' (default) | 'orthographic'
%
%   <xdir> determines the initial x-dir setting.
%     'normal' (default) | 'reverse'
%
%   <lighting> determines the initial lighting setting.
%     'none' | 'flat' (default) | 'gouraud'
%
%   <lightpos> determines the initial light position setting.
%     'headlight' (default) | 'right' | 'left'
%
%   <background> determines the initial background setting.
%     'red' | 'green' | 'blue' | 
%     'cyan' | 'magenta' | 'yellow' | 'black' (default) | 
%     'white' | 'gray' | 'light gray'
%
%   <edges> determines the initial edges setting
%   for all applicable surfaces.
%     'off' (default) | 'on'
%
%   <markers> determines the initial markers setting
%   for all applicable surfaces.
%     'off' (default) | 'on'
%
%   <visibility> determines the initial visibility setting
%   for all regular surfaces.
%     'off' | 'on' (default)
%
%   <braincolor> determines the initial colormap setting for
%   the brain surface.
%     'red' | 'green' | 'blue' | 
%     'cyan' | 'magenta' | 'yellow' | 'black' | 
%     'white' (default) | 'gray'
%
%   <surfacecolor> determines the initial colormap setting for
%   the regular surfaces.
%     <user-defined colormap> | 'autumn' | 'bone' | 'colorcube' |
%     'cool' | 'copper' | 'flag' | 'gray' | 'hot' (default) |
%     'hsv' | 'jet' | 'lines' | 'pink' | 'prism' | 'spring' |
%     'summer' | 'vga' | 'white' | 'winter'
%
%   <selectioncolor> determines the initial colormap setting for
%   the selection surface.
%     'red' | 'green' (default) | 'blue' | 
%     'cyan' | 'magenta' | 'yellow' | 'black' | 
%     'white' | 'gray'
%
%   <curvaturecolor> determines the initial colormap setting for
%   the curvature surface.
%     'red' | 'green' | 'blue' (default) | 
%     'cyan' | 'magenta' | 'yellow' | 'black' | 
%     'white' | 'gray'
%
%   <partialvolumecolor> determines the initial colormap setting for
%   the partial-volume voxel indicators.
%     'red' | 'green' | 'blue' | 
%     'cyan' (default) | 'magenta' | 'yellow' | 'black' | 
%     'white' | 'gray'
%
%   <voxelboundcolor> determines the initial colormap setting for
%   the voxel boundaries surface.
%     'red' | 'green' | 'blue' | 
%     'cyan' (default) | 'magenta' | 'yellow' | 'black' | 
%     'white' | 'gray'
%
%   <pvaluedir> determines the initial p-value threshold 
%   inequality for all applicable surfaces except the 
%   curvature surface.
%     '<' | '<=' (default) | '>' | '>='
%
%   <pvalue> determines the initial p-value threshold for
%   all applicable surfaces except the curvature surface.
%     any number (default is 1)
%
%   <offset> determines how surfaces are initially offset:
%     'zeros' (default) | 'successive'
%   'zeros' means all surfaces to have offset 0
%   'successive' means surfaces to have successive offsets 1,2,3,etc.
%
%   <boundary> determines the initial boundary setting for all
%   applicable surfaces.  (the 'outline' option is not listed
%   here since it is particular to selection surfaces.)
%     'expand' (default) | 'restrict'
%
%   <redrawmode> determines rendering behavior:
%     'manual' (default) | 'auto'
%
%   <snapshotmethod> determines how to obtain snapshots of
%   the render window:
%     'getframe' (default) | 'print'
%
%   note that preference changes take effect only on the next 
%   call to viewsurfacedata.

plist = {'colormaps' 'viewangle' 'projection' 'xdir' 'lighting' 'lightpos' 'background' 'edges' ...
         'markers' 'visibility' 'braincolor' 'surfacecolor' 'selectioncolor' ...
         'curvaturecolor' 'partialvolumecolor' 'voxelboundcolor' ...
         'pvaluedir' 'pvalue' 'offset' 'boundary' 'redrawmode' 'snapshotmethod'};

if exist('prefs','var') && isequal(prefs,'default')
  f = viewsurfacedata_loadsettings(plist,1);
end

if ~exist('prefs','var')
  f = viewsurfacedata_loadsettings(plist,0);
end

if exist('prefs','var') && ~isequal(prefs,'default')
  % verify but don't transform
  try
    viewsurfacedata_transformpreferences(prefs);
  catch
    fprintf(1,['error: ',chopline(lasterr),'\n']);
    return;
  end
  % save them
  for p=1:length(plist)
    settings.(plist{p}) = prefs.(plist{p});
  end
  viewsurfacedata_savesettings(settings);
  % report
  fprintf(1,'preferences have been saved and will take effect on the next call to viewsurfacedata.\n');
end
