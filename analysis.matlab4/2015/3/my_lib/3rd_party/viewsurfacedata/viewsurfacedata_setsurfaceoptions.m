function viewsurfacedata_setsurfaceoptions(surface,options)

% function viewsurfacedata_setsurfaceoptions(surface,options)
%
% <surface> is:
%   (1) N (positive integer) for the regular surfaces
%   (2) 'sN' for the selection surfaces
%   (3) a vector or cell vector of (1), or cell vector of (2)
% <options> is:
%   {<edges> <markers> <visibility> <cmap> <alpha> <pvaluedir> <pvalue> <offset> <boundarytype>}
%     where <edges> is 0/1
%           <markers> is 0/1
%           <visibility> is 0/1
%           <cmap> is like 'hot' or 'green'
%           <alpha> is in [.01,1]
%           <pvaluedir> is 1 (<), 2 (<=), 3 (>), or 4 (>=)
%           <pvalue> is a number
%           <offset> is >= 0
%           <boundarytype> is 0 (expand), 1 (restrict), 2 (outline)
%     and where any element being [] means to don't change
%
% this function sets the options for the surface(s) indicated
% by <surface>.  if an option does not apply to the surface,
% we quietly ignore that option (but still check syntax).
%
% we force a redraw of the render window if 'auto'.
%
% note: you cannot set the colormap for multiple surfaces of different types.

global VS_GUI;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% deal with surface
if ~iscell(surface)
  surface = num2cell(surface);
end
stypes = [];
for p=1:length(surface)
  s = surface{p};
  if all(isint(s))
    assert(s>=1 && s<=length(handles.rsurf),'invalid <surface>');
    surface{p} = handles.rsurf(s);
    stype = 0;
    stypes = union(stypes,stype);
  else
    if ischar(s)
      assert(s(1)=='s','invalid <surface>');
      s(1) = [];
      s = str2double(s);
      assert(s>=1 && s<=length(handles.ssurf),'invalid <surface>');
      surface{p} = handles.ssurf(s);
      stype = 1;
      stypes = union(stypes,stype);
    else
      assert(0,'invalid <surface>');
    end
  end
end

% deal with options
edges = options{1};
markers = options{2};
visibility = options{3};
cmap = options{4};
alpha = options{5};
pvaluedir = options{6};
pvalue = options{7};
offset = options{8};
boundarytype = options{9};

% weird check to do FIRST
if length(stypes) > 1 && ~isempty(cmap)
  fprintf('error: you cannot set a colormap for surfaces of different types.\n');
  return;
end

% check options
assert(isempty(edges) || edges==0 || edges==1,'invalid <edges> option');
assert(isempty(markers) || markers==0 || markers==1,'invalid <markers> option');
assert(isempty(visibility) || visibility==0 || visibility==1,'invalid <visibility> option');
if ~isempty(cmap)
  switch stype
  case 0
    temp = handles.cmaps;
  case 1
    temp = listcolornames;
  end
  cmap = find(ismember(temp,cmap));
  assert(~isempty(cmap),'invalid <cmap> option');
  cmap = firstel(cmap);
end
assert(isempty(alpha) || isfinitenum(alpha) && alpha>=.01 && alpha<=1,'invalid <alpha> option');
assert(isempty(pvaluedir) || all(isint(pvaluedir)) && pvaluedir>=1 && pvaluedir<=4,'invalid <pvaluedir> option');
assert(isempty(pvalue) || isfinitenum(pvalue),'invalid <pvalue> option');
assert(isempty(offset) || isfinitenum(offset) && offset>=0,'invalid <offset> option');
assert(isempty(boundarytype) || all(isint(boundarytype)) && boundarytype>=0 && boundarytype<=2,'invalid <boundarytype> option');

% do it
for p=1:length(surface)
  s = surface{p};
  if ~isempty(edges)
    handles.surfacerecord{s}.edges = edges;
  end
  if ~isempty(markers)
    handles.surfacerecord{s}.markers = markers;
  end
  if ~isempty(visibility)
    handles.surfacerecord{s}.visibility = visibility;
    handles.draworderchanged = 1;
  end
  if ~isempty(cmap)
    handles.surfacerecord{s}.colormap = cmap;
  end
  if ~isempty(alpha)
    handles.surfacerecord{s}.alpha = alpha;
  end
  if ~isempty(pvaluedir) && stype==0
    handles.surfacerecord{s}.pvaluereverse = pvaluedir;
    handles.surfacerecord{s}.pvaluechanged = 1;
    if ~ismember(s,handles.psurf)
      handles.draworderchanged = 1;
    end
  end
  if ~isempty(pvalue) && stype==0
    handles.surfacerecord{s}.pvalue = pvalue;
    handles.surfacerecord{s}.pvaluechanged = 1;
    if ~ismember(s,handles.psurf)
      handles.draworderchanged = 1;
    end
  end
  if ~isempty(offset)
    handles.surfacerecord{s}.offset = offset;
    handles.surfacerecord{s}.offsetchanged = 1;
    handles.draworderchanged = 1;
  end
  if ~isempty(boundarytype)
    if handles.surfacerecord{s}.boundary==3 || boundarytype==3-1
      handles.draworderchanged = 1;
    end
    handles.surfacerecord{s}.boundary = boundarytype+1;
    handles.surfacerecord{s}.boundarychanged = 1;
  end
end

% update surface selector
viewsurfacedata_gui('surface_Callback',handles.surface,[],handles);

% redraw if necessary
if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  viewsurfacedata_gui('redraw_Callback',handles.redraw,[],handles);
end

% report
fprintf(1,'surface options successfully set.\n');
