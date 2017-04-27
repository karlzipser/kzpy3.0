function prefs = viewsurfacedata_transformpreferences(prefs)

% function prefs = viewsurfacedata_transformpreferences(prefs)
%
% <prefs> is a prefs (or settings) struct.
%
% return <prefs> but with the fields that correspond to
% preferences transformed into the internal format.  any
% unexpected preference value triggers an error, so you
% can use me to verify preference values.

assert(iscell(prefs.colormaps),'<colormaps> must be a cell vector');
for p=1:length(prefs.colormaps)
  assert(ischar(prefs.colormaps{p}),'elements of <colormaps> must be strings');
end
assert(isscalar(prefs.viewangle) && isfinitenum(prefs.viewangle) && prefs.viewangle > 0,'invalid <viewangle> value');
switch prefs.projection
case 'perspective'
  prefs.projection = 1;
case 'orthographic'
  prefs.projection = 2;
otherwise
  error('invalid <projection> value');
end
switch prefs.xdir
case 'normal'
  prefs.xdir = 1;
case 'reverse'
  prefs.xdir = 2;
otherwise
  error('invalid <xdir> value');
end
switch prefs.lighting
case 'none'
  prefs.lighting = 1;
case 'flat'
  prefs.lighting = 2;
case 'gouraud'
  prefs.lighting = 3;
otherwise
  error('invalid <lighting> value');
end
switch prefs.lightpos
case 'headlight'
  prefs.lightpos = 1;
case 'right'
  prefs.lightpos = 2;
case 'left'
  prefs.lightpos = 3;
otherwise
  error('invalid <lightpos> value');
end
assert(iscolorname(prefs.background),'invalid <background> value');
switch prefs.edges
case 'off'
  prefs.edges = 0;
case 'on'
  prefs.edges = 1;
otherwise
  error('invalid <edges> value');
end
switch prefs.markers
case 'off'
  prefs.markers = 0;
case 'on'
  prefs.markers = 1;
otherwise
  error('invalid <markers> value');
end
switch prefs.visibility
case 'off'
  prefs.visibility = 0;
case 'on'
  prefs.visibility = 1;
otherwise
  error('invalid <visibility> value');
end
assert(iscolorname(prefs.braincolor),'invalid <braincolor> value');
assert(ischar(prefs.surfacecolor),'invalid <surfacecolor> value');  % minimal check
assert(iscolorname(prefs.selectioncolor),'invalid <selectioncolor> value');
assert(iscolorname(prefs.curvaturecolor),'invalid <curvaturecolor> value');
assert(iscolorname(prefs.partialvolumecolor),'invalid <partialvolumecolor> value');
assert(iscolorname(prefs.voxelboundcolor),'invalid <voxelboundcolor> value');
switch prefs.pvaluedir
case '<'
  prefs.pvaluedir = 1;
case '<='
  prefs.pvaluedir = 2;
case '>'
  prefs.pvaluedir = 3;
case '>='
  prefs.pvaluedir = 4;
otherwise
  error('invalid <pvaluedir> value');
end
assert(isfinitenum(prefs.pvalue),'invalid <pvalue> value');
switch prefs.offset
case 'zeros'
  prefs.offset = 1;
case 'successive'
  prefs.offset = 2;
otherwise
  error('invalid <offset> value');
end
switch prefs.boundary
case 'expand'
  prefs.boundary = 1;
case 'restrict'
  prefs.boundary = 2;
otherwise
  error('invalid <boundary> value');
end
switch prefs.redrawmode
case 'manual'
  prefs.redrawmode = 1;
case 'auto'
  prefs.redrawmode = 2;
otherwise
  error('invalid <redrawmode> value');
end
switch prefs.snapshotmethod
case 'getframe'
  prefs.snapshotmethod = 1;
case 'print'
  prefs.snapshotmethod = 2;
otherwise
  error('invalid <snapshotmethod> value');
end
