function varargout = viewsurfacedata_gui(varargin)

% function varargout = viewsurfacedata_gui(varargin)
%
% varargin:
%   fig: handle of the figure window
%   surfs: vector of surface handles
%   pts: [pt1 pt1tip pt2 pt2tip] of handles
%
% VIEWSURFACEDATA_GUI M-file for viewsurfacedata_gui.fig
%      VIEWSURFACEDATA_GUI, by itself, creates a new VIEWSURFACEDATA_GUI or raises the existing
%      singleton*.
%
%      H = VIEWSURFACEDATA_GUI returns the handle to a new VIEWSURFACEDATA_GUI or the handle to
%      the existing singleton*.
%
%      VIEWSURFACEDATA_GUI('Property','Value',...) creates a new VIEWSURFACEDATA_GUI using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to viewsurfacedata_gui_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      VIEWSURFACEDATA_GUI('CALLBACK') and VIEWSURFACEDATA_GUI('CALLBACK',hObject,...) call the
%      local function named CALLBACK in VIEWSURFACEDATA_GUI.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help viewsurfacedata_gui

% Last Modified by GUIDE v2.5 09-Oct-2004 12:33:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @viewsurfacedata_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @viewsurfacedata_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Outputs from this function are returned to the command line.
function varargout = viewsurfacedata_gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION

% --- Executes just before viewsurfacedata_gui is made visible.
function viewsurfacedata_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for viewsurfacedata_gui
handles.output = hObject;

global VS_R VS_RPTR VS_TCOLORS VS_RVNUM VS_SELECTION;

% deal with initializations that didn't already happen
VS_SELECTION = {};
VS_OUTLINEC = [];
VS_OUTLINEE = [];  % this should actually be a sparse matrix, but doesn't matter
VS_OUTLINEV = [];

% default is [].  see *_importcamerapreset.m.
handles.importcamerapreset = [];

% do we have epi information?  (important to do early!)
handles.epion = ~isempty(VS_RPTR{1});

% do we have curvature information?
handles.curvon = ismember(1,VS_TCOLORS);

% handle input
handles.fig = varargin{1};  % saving this is redundant, but oh well
handles.surfs = varargin{2};
handles.ptclick = varargin{3};
handles.pvv = [];  % there is no detectable difference between the pvv not being drawn yet and there being no pvv.  oh well.
handles.ptclickdata = NaN*zeros(1,3);
handles.ax = get(handles.fig,'CurrentAxes');
handles.lt = findobj(handles.fig,'Type','light');

% note that surfaces are like:
%   1      (brain)
%   numreg (regular)
%   1      (selection) (this may expand, however)
%   1      (curvature)
%   1      (partial-volume)
%   1/3    (voxel boundaries)

% deal with *surf (these are indices into surfs!)
handles.bsurf = 1;
if handles.epion && handles.curvon
  handles.rsurf = 2:length(handles.surfs)-6;
  handles.ssurf = length(handles.surfs)-5;
  handles.csurf = length(handles.surfs)-4;
  handles.psurf = length(handles.surfs)-3;
  handles.vsurf = length(handles.surfs)-2:length(handles.surfs);
end
if handles.epion && ~handles.curvon
  handles.rsurf = 2:length(handles.surfs)-5;
  handles.ssurf = length(handles.surfs)-4;
  handles.csurf = [];
  handles.psurf = length(handles.surfs)-3;
  handles.vsurf = length(handles.surfs)-2:length(handles.surfs);
end
if ~handles.epion && handles.curvon
  handles.rsurf = 2:length(handles.surfs)-2;
  handles.ssurf = length(handles.surfs)-1;
  handles.csurf = length(handles.surfs);
  handles.psurf = [];
  handles.vsurf = [];
end
if ~handles.epion && ~handles.curvon
  handles.rsurf = 2:length(handles.surfs)-1;
  handles.ssurf = length(handles.surfs);
  handles.csurf = [];
  handles.psurf = [];
  handles.vsurf = [];
end

% crazy draworder stuff (note that the partial volume stuff is exempt here)
handles.draworder = [];  % to be initialized
handles.draworderchanged = 1;  % this tracks changes in layerorder (not currently possible), offset, visibility, p-value, epiclicked, vclicked
handles.layerorder = [handles.rsurf handles.ssurf handles.csurf];
handles.layerorderchanged = 0;  % this is 0 since we initialized the surfaces in the correct order

% internal constants/calculations
[handles.edgealphav,handles.bcolors,handles.scolors,handles.ccolors, ...
 handles.pcolors,handles.icolors,handles.bgcolors,handles.cmaps] = ...   %,handles.settingsfile
  viewsurfacedata_constants('edgealphav','bcolors','scolors','ccolors', ...
                            'pcolors','icolors','bgcolors','cmaps');  % ,'settingsfile'

% load settings, transforming the preference settings into internal format
handles.settings = viewsurfacedata_transformpreferences(viewsurfacedata_loadsettings({},0));

% add custom colormaps from the user-defined settings
handles.cmaps = [handles.settings.colormaps handles.cmaps];

% initialize other records
handles.cpqueue = {};
handles.func = 'disp(mat2str($VOXEL$))';

% initialize camera stuff
handles.cameraorig = {campos(handles.ax) camproj(handles.ax) camtarget(handles.ax) camup(handles.ax) camva(handles.ax)};
handles.cameraresettoorig = 0;
camerapreset_reset(handles);

% initialize surface selector
ss = { 'brain surface' };
for p=1:length(handles.rsurf)
  ss = [ss; { ['surface ',num2str(p)] }];
end
ss = [ss; { 'selection surface 1' }];
if handles.curvon
  ss = [ss; { 'curvature surface' }];
end
if handles.epion
  ss = [ss; { 'partial-volume voxels' }];
  ss = [ss; { 'voxel boundaries' }];
end
set(handles.surface,'String',ss);
set(handles.surface,'Value',1);

% initialize selection surface selector
sss = { 'selection surface 1' };
set(handles.selectionsurface,'String',sss);
set(handles.selectionsurface,'Value',1);

% initialize background selector strings
set(handles.lightbackground,'String',viewsurfacedata_constants('colornames'));

% initialize status string
set(handles.status,'String','status: ready');

% some initializations based on preferences
handles.viewangleval = handles.settings.viewangle;
set(handles.viewangle,'String',num2str(handles.viewangleval));
set(handles.projection,'Value',handles.settings.projection);
set(handles.xdir,'Value',handles.settings.xdir);
set(handles.lighttype,'Value',handles.settings.lighting);
set(handles.lightpos,'Value',handles.settings.lightpos);
set(handles.lightbackground,'Value',colorname2num(handles.settings.background));
switch handles.settings.redrawmode  % we call manual_Callback later on
case 1
  set(handles.manual,'String','auto');  % note the reversal (since we simulate a click later on)
case 2
  set(handles.manual,'String','manual');
end

% deal with default regular surface colormap
temp = find(ismember(handles.cmaps,handles.settings.surfacecolor));
if isempty(temp)
  fprintf(1,'warning: the default colormap setting for regular surfaces is not one of the known colormaps.  defaulting to the hot colormap.\n');
  temp = find(ismember(handles.cmaps,'hot'));
end
assert(~isempty(temp));
handles.colormapdefaultnum = firstel(temp);  % hacky to store it, oh well

% initialize surface record (indices are the same as those into surfs, except for the
% voxel boundaries case which gets collapsed from the last 3 -> last 1.  indices also
% correspond to the GUI interface surface drop-down, which is natural and good.)
for p=1:length(ss)
  handles.surfacerecord{p} = createsurfacerecord(p,handles);
end
% handle some explicit overrides
  % brain surface
handles.surfacerecord{handles.bsurf}.colormap = colorname2num(handles.settings.braincolor);
handles.surfacerecord{handles.bsurf}.visibility = 1;
  % selection surface (this assumes there is only one ssurf to start with)
handles.surfacerecord{handles.ssurf}.colormap = colorname2num(handles.settings.selectioncolor);
handles.surfacerecord{handles.ssurf}.visibility = 1;
if handles.curvon  % curvature surface
  handles.surfacerecord{handles.csurf}.colormap = colorname2num(handles.settings.curvaturecolor);
  handles.surfacerecord{handles.csurf}.visibility = 0;
  handles.surfacerecord{handles.csurf}.pvaluereverse = 4;
  handles.surfacerecord{handles.csurf}.pvalue = 1;  % TODO: perhaps revisit?
end
if handles.epion
  % partial-volume voxels
  handles.surfacerecord{handles.psurf}.edges = 1;
  handles.surfacerecord{handles.psurf}.markers = 1;
  handles.surfacerecord{handles.psurf}.visibility = 0;
  handles.surfacerecord{handles.psurf}.colormap = colorname2num(handles.settings.partialvolumecolor);
  handles.surfacerecord{handles.psurf}.offset = 0;
  handles.surfacerecord{handles.psurf}.offsetchanged = 0;
  % intersection surface
  handles.surfacerecord{handles.vsurf(1)}.colormap = colorname2num(handles.settings.voxelboundcolor);
  handles.surfacerecord{handles.vsurf(1)}.edges = -1;  % TACKY
  handles.surfacerecord{handles.vsurf(1)}.visibility = 0;
end

% initialize epiclicked/vclicked
if handles.epion
  handles.xyzbegin = VS_R{VS_RPTR{1}}.xyzbegin;  % TODO: should make more use of these instead of VS_R{VS_RPTR{1}} directly
  handles.dsize = VS_R{VS_RPTR{1}}.xyzsize;
  handles.epiclicked{handles.ssurf} = zeros(handles.dsize);  % boolean matrix of selected epi voxels
end
  % must be 1 so that VS_SELECTION is calced initially (since there may be an existing value from earlier)!
handles.clickedchanged{handles.ssurf} = 1;
handles.vclicked{handles.ssurf} = zeros(1,VS_RVNUM);  % boolean vector of selected vertices

% update some more
if ~handles.epion
  set(handles.selectionmode,'Enable','off');
  set(handles.selectionfunction,'Enable','off');
end

% update the manual button stuff
manual_Callback(handles.manual, eventdata, handles);

% update the surface selector dependencies
surface_Callback(handles.surface, eventdata, handles);

% update the selection surface selector dependencies
selectionsurface_Callback(handles.selectionsurface, eventdata, handles);

% set up figure linkage (the 0 is just a dummy value)
set(handles.fig,'WindowButtonDownFcn', ...
    ['viewsurfacedata_gui(''selectionclick_Callback'',0,[],guidata(',num2str(hObject,64),'))']);  % HACK: can this be fixed?

guidata(handles.output, handles);

% UIWAIT makes viewsurfacedata_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION-RELATED

function f = createsurfacerecord(p,handles)

% <p> is the surfacerecord number
% <handles> is as usual
%
% return a default surfacerecord struct.

f.edges = handles.settings.edges;
f.markers = handles.settings.markers;
f.visibility = handles.settings.visibility;
f.colormap = handles.colormapdefaultnum;
f.alpha = 1;
f.pvaluereverse = handles.settings.pvaluedir;
f.pvalue = handles.settings.pvalue;
f.pvaluechanged = 0;
switch handles.settings.offset
case 1
  f.offset = 0;
case 2
  f.offset = p-1;  % fortuitously, the brain invariably gets 0 here
end
f.offsetchanged = f.offset~=0;  % force an initial update if necessary
f.boundary = handles.settings.boundary;  % 1 means expand, 2 means restrict, 3 means outline (but 3 is not a preference setting)
f.boundarychanged = 0;
f.selectionmode = choose(handles.epion,1,2);  % this actually isn't used for non-selection surfaces

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CAMERA-RELATED

function camerareset_Callback(hObject, eventdata, handles)
handles.cameraresettoorig = 1;
redraw_Callback(handles.redraw, eventdata, handles);  % force immediate redraw

%%%%%%%%%%%%%%%

function camerapreset_Callback(hObject, eventdata, handles)
camerapreset_update(handles);
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function camerapresetsave_Callback(hObject, eventdata, handles)

% if no preset selected
if get(handles.camerapreset,'Value')==1
  set(handles.status,'String','status: waiting for text input...'); drawnow;
  in = input('\nenter a name for the preset to be saved -> ','s');
  set(handles.status,'String','status: ready');
  if isempty(in)
    fprintf(1,'empty input detected, so doing nothing.\n');
    return;
  end
  num = size(handles.settings.camerapresets,1)+1;
% otherwise, some preset selected
else
  num = get(handles.camerapreset,'Value')-1;
  in = handles.settings.camerapresets{num,1};
end

handles.settings.camerapresets{num,1} = in;
handles.settings.camerapresets{num,2} = ...
  {campos(handles.ax) camproj(handles.ax) camtarget(handles.ax) camup(handles.ax) camva(handles.ax) choose(get(handles.xdir,'Value')==1,'normal','reverse')};
temp.camerapresets = handles.settings.camerapresets;
viewsurfacedata_savesettings(temp);

if get(handles.camerapreset,'Value')==1
  fprintf(1,'current camera view successfully saved as a new preset named ''%s''.\n',in);
  camerapreset_reset(handles);
else
  fprintf(1,'current camera view successfully saved to an existing preset named ''%s''.\n',in);
  % no need to call camerapreset_reset since the name stayed the same
end

guidata(handles.output, handles);

%%%%%%%%%%%%%%%

function camerapresetdelete_Callback(hObject, eventdata, handles)
idx = get(handles.camerapreset,'Value') - 1;  % idx to delete
idxname = handles.settings.camerapresets{idx,1};
handles.settings.camerapresets(idx,:) = [];
temp.camerapresets = handles.settings.camerapresets;
viewsurfacedata_savesettings(temp);
fprintf(1,'\ncamera preset named ''%s'' successfully deleted.\n',idxname);
camerapreset_reset(handles);
guidata(handles.output, handles);

%%%%%%%%%%%%%%%

function camerapresetrename_Callback(hObject, eventdata, handles)

set(handles.status,'String','status: waiting for text input...'); drawnow;
in = input('\nenter a new name for the preset -> ','s');
set(handles.status,'String','status: ready');

if isempty(in)
  fprintf(1,'empty input detected, so doing nothing.\n');
  return;
end

idx = get(handles.camerapreset,'Value') - 1;  % idx to rename
idxname = handles.settings.camerapresets{idx,1};
handles.settings.camerapresets{idx,1} = in;
temp.camerapresets = handles.settings.camerapresets;
viewsurfacedata_savesettings(temp);
fprintf(1,'camera preset named ''%s'' successfully renamed to ''%s''.\n',idxname,in);
camerapreset_reset(handles);
guidata(handles.output, handles);

%%%%%%%%%%%%%%%

function rotup_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function rotleft_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function rollccw_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function scrollup_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function scrollleft_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function zoom_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
if isnan(val) || val<=0
  set(hObject,'String','1');
end
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function viewangle_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
if isnan(val) || val<=0
  set(hObject,'String',num2str(handles.viewangleval));
end
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function projection_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function xdir_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LIGHT-RELATED

function lighttype_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function lightpos_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function lightbackground_Callback(hObject, eventdata, handles)
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SELECTION-RELATED

function selectionmode_Callback(hObject, eventdata, handles)

global VS_RVNUM;

val = get(handles.selectionsurface,'Value');
sval = handles.ssurf(val);
switch get(hObject,'String')
case 'vertex'
  dindices = viewsurfacedata_lookup(-1,find(handles.vclicked{sval}));
  handles.epiclicked{sval} = zeros(handles.dsize);
  handles.epiclicked{sval}(dindices) = 1;
  set(hObject,'String','voxel');
  handles.surfacerecord{sval}.selectionmode = 1;
  fprintf(1,'\nselection surface %d converted from vertices to voxels.\n',val);
case 'voxel'
  [dummy,good] = viewsurfacedata_lookup(-1,-1,handles.epiclicked{sval});
  handles.vclicked{sval} = zeros(1,VS_RVNUM);
  handles.vclicked{sval}(good) = 1;
  set(hObject,'String','vertex');
  handles.surfacerecord{sval}.selectionmode = 2;
  fprintf(1,'\nselection surface %d converted from voxels to vertices.\n',val);
end
handles.clickedchanged{sval} = 1;
handles.draworderchanged = 1;

% force a redraw
redraw_Callback(handles.redraw, eventdata, handles);

%%%%%%%%%%%%%%%

function selectionsurface_Callback(hObject, eventdata, handles)

% we do not save handles since there is no need to.

val = get(handles.selectionsurface,'Value');
sval = handles.ssurf(val);
switch handles.surfacerecord{sval}.selectionmode
case 1
  set(handles.selectionmode,'String','voxel');
case 2
  set(handles.selectionmode,'String','vertex');  % (note that this may be disabled when ~handles.epion (see above))
end
set(handles.selectionsurfacedelete,'Enable',choose(length(handles.ssurf)==1,'off','on'));

%%%%%%%%%%%%%%%

function selectionsurfaceadd_Callback(hObject, eventdata, handles)

global VS_FIG VS_INTERVAL VS_SPECIALSV VS_INTERVAL VS_BEGINVALUE VS_RVNUM;

% get handle of new surface
prev = get(0,'CurrentFigure');  % a little hacky, oh well
set(0,'CurrentFigure',VS_FIG);
hold on;
  % i think we can pass [] as below and 2 as boundary and 1 as p,
  % since we are always empty to start with, and any change with
  % trigger a redraw (and therefore call to _drawsurface)
ns = viewsurfacedata_drawsurface(3,[],1,[],[],1,2,2);
hold off;
set(0,'CurrentFigure',prev);

% insert handle into .surfs
handles.surfs = insertelt(handles.surfs,handles.ssurf(end)+1,ns);

% add to .ssurf
handles.ssurf = [handles.ssurf handles.ssurf(end)+1];

% shift the subsequent ones
handles.csurf = handles.csurf+1;
handles.psurf = handles.psurf+1;
handles.vsurf = handles.vsurf+1;

% set some flags
handles.draworderchanged = 1;
handles.layerorder = [handles.rsurf handles.ssurf handles.csurf];
handles.layerorderchanged = 1;

% update surfacerecord
nsr = createsurfacerecord(1,handles);  % we pass 1 here to force 0 offset
  % this repeats code from the init routine, oh well:
nsr.colormap = colorname2num(handles.settings.selectioncolor);
nsr.visibility = 1;
handles.surfacerecord = insertelt(handles.surfacerecord,handles.ssurf(end),{nsr});

% update other (this mirrors the initialization)
if handles.epion
  handles.epiclicked{handles.ssurf(end)} = zeros(handles.dsize);
end
handles.clickedchanged{handles.ssurf(end)} = 1;
handles.vclicked{handles.ssurf(end)} = zeros(1,VS_RVNUM);

% deal with color (note that the axis limits are re-set on the next redraw)
VS_SPECIALSV{end+1} = VS_SPECIALSV{end}-VS_INTERVAL;  % tack on a new one
VS_BEGINVALUE = VS_SPECIALSV{end}-VS_INTERVAL/2;  % update the beginning value

% update surface drop-down
ss = get(handles.surface,'String');
ssv = get(handles.surface,'Value');
if ssv >= handles.ssurf(end)  % update value first
  set(handles.surface,'Value',ssv+1);
end
set(handles.surface,'String',insertelt(ss,handles.ssurf(end),{sprintf('selection surface %d',length(handles.ssurf))}));

% update selection surface drop-down
sss = get(handles.selectionsurface,'String');
set(handles.selectionsurface,'String',insertelt(sss,length(handles.ssurf),{sprintf('selection surface %d',length(handles.ssurf))}));
set(handles.selectionsurface,'Value',length(handles.ssurf));  % here we auto-switch

% update selection surface dependencies
selectionsurface_Callback(handles.selectionsurface, eventdata, handles);

% save handles
guidata(handles.output, handles);

%%%%%%%%%%%%%%%

function selectionsurfacedelete_Callback(hObject, eventdata, handles)

global VS_SPECIALSV VS_INTERVAL VS_BEGINVALUE VS_SELECTION;

val = get(handles.selectionsurface,'Value');
sval = handles.ssurf(val);

% delete graphics object
delete(handles.surfs(sval));

% remove handle from .surfs
handles.surfs(sval) = [];

% remove from .ssurf, shift subsequent
handles.ssurf(val) = [];
handles.ssurf(val:end) = handles.ssurf(val:end)-1;

% shift the subsequent ones
handles.csurf = handles.csurf-1;
handles.psurf = handles.psurf-1;
handles.vsurf = handles.vsurf-1;

% set some flags
handles.draworderchanged = 1;
handles.layerorder = [handles.rsurf handles.ssurf handles.csurf];
handles.layerorderchanged = 1;

% update surfacerecord
handles.surfacerecord = deleteelt(handles.surfacerecord,sval);

% update other
if handles.epion
  handles.epiclicked = deleteelt(handles.epiclicked,sval);
end
handles.clickedchanged = deleteelt(handles.clickedchanged,sval);
handles.vclicked = deleteelt(handles.vclicked,sval);

% deal with VS_SELECTION
VS_SELECTION = deleteelt(VS_SELECTION,sval);

% deal with color (note that the axis limits are re-set on the next redraw)
VS_SPECIALSV = deleteelt(VS_SPECIALSV,val);
for p=val:length(VS_SPECIALSV)
  VS_SPECIALSV{p} = VS_SPECIALSV{p}+VS_INTERVAL;  % shift back
end
VS_BEGINVALUE = VS_SPECIALSV{end}-VS_INTERVAL/2;  % update the beginning value

% we have to redraw the subsequent selection surfaces since their corresponding
% VS_SPECIALSV value changed.  and we have to redraw all surfaces because the
% below mechanism makes it such that the underlying values for any given surface
% may need to be updated.  it turns out that since we set handles.draworderchanged
% to 1, these redraws will be triggered on the next redraw anyway.  so good.

% update surface drop-down
ss = get(handles.surface,'String');
ssv = get(handles.surface,'Value');
if ssv >= sval  % update value first
  set(handles.surface,'Value',max(ssv-1,1));
end
ss = deleteelt(ss,sval);
for p=sval:handles.ssurf(end)
  ss{p} = sprintf('selection surface %d',p-handles.ssurf(1)+1);  % hacky!
end
set(handles.surface,'String',ss);

% update surface dependencies (since the current selection might have changed)
surface_Callback(handles.surface, eventdata, handles);

% update selection surface drop-down
sss = get(handles.selectionsurface,'String');
sssv = get(handles.selectionsurface,'Value');
if sssv >= val  % update value first
  set(handles.selectionsurface,'Value',max(sssv-1,1));
end
sss = deleteelt(sss,val);
for p=val:length(sss)
  sss{p} = sprintf('selection surface %d',p);
end
set(handles.selectionsurface,'String',sss);

% update selection surface dependencies
selectionsurface_Callback(handles.selectionsurface, eventdata, handles);

% force a redraw since we called delete() above
redraw_Callback(handles.redraw, eventdata, handles);

%%%%%%%%%%%%%%%

function selectionsurfaceoutline_Callback(hObject, eventdata, handles)

global VS_RVNUM;

val = get(handles.selectionsurface,'Value');
sval = handles.ssurf(val);
switch get(handles.selectionmode,'String')
case 'vertex'
  prev = length(find(handles.vclicked{sval}));
  [dummy,edgevindices] = viewsurfacedata_outline(find(handles.vclicked{sval}));
  handles.vclicked{sval} = zeros(1,VS_RVNUM);
  handles.vclicked{sval}(edgevindices) = 1;
  cur = length(edgevindices);
  fprintf(1,'\nselection surface %d converted to outline, reducing from %d vertices to %d vertices.\n',val,prev,cur);
case 'voxel'
  [dummy,good] = viewsurfacedata_lookup(-1,-1,handles.epiclicked{sval});
  prev = length(good);
  [dummy,edgevindices] = viewsurfacedata_outline(good);
  handles.vclicked{sval} = zeros(1,VS_RVNUM);
  handles.vclicked{sval}(edgevindices) = 1;
  cur = length(edgevindices);
  fprintf(1,'\nselection surface %d converted to outline, reducing from %d vertices to %d vertices.\nnote that in the course of doing so, we switched to vertex mode.\n',val,prev,cur);
  set(handles.selectionmode,'String','vertex');
  handles.surfacerecord{sval}.selectionmode = 2;
end
handles.clickedchanged{sval} = 1;
handles.draworderchanged = 1;

% force a redraw
redraw_Callback(handles.redraw, eventdata, handles);

%%%%%%%%%%%%%%%

function selectiondraw_Callback(hObject, eventdata, handles)
switch get(hObject,'String')
case 'off'
  set(hObject,'String','draw');
case 'draw'
  set(hObject,'String','erase');
case 'erase'
  set(hObject,'String','off');
end

%%%%%%%%%%%%%%%

function selectionhide_Callback(hObject, eventdata, handles)
switch get(hObject,'String')
case 'hide'
  set(hObject,'String','show');
case 'show'
  set(hObject,'String','hide');
end
if ~isequal(get(handles.manual,'String'),'manual')
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function selectionclear_Callback(hObject, eventdata, handles)

global VS_RVNUM;

val = get(handles.selectionsurface,'Value');
sval = handles.ssurf(val);
switch handles.surfacerecord{sval}.selectionmode
case 1
  handles.epiclicked{sval} = zeros(handles.dsize);
case 2
  handles.vclicked{sval} = zeros(1,VS_RVNUM);
end
handles.clickedchanged{sval} = 1;
handles.draworderchanged = 1;
fprintf(1,'\nselection surface %d has been cleared!\n',val);
if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function selectionunclear_Callback(hObject, eventdata, handles)

global VS_R VS_RPTR VS_RVNUM;

val = get(handles.selectionsurface,'Value');
sval = handles.ssurf(val);
switch handles.surfacerecord{sval}.selectionmode
case 1
  handles.epiclicked{sval} = zeros(handles.dsize);
  handles.epiclicked{sval}(VS_R{VS_RPTR{1}}.indices) = 1;
case 2
  handles.vclicked{sval} = ones(1,VS_RVNUM);
end
handles.clickedchanged{sval} = 1;
handles.draworderchanged = 1;
fprintf(1,'\nselection surface %d has been filled!\n',val);

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function selectionline_Callback(hObject, eventdata, handles)

% this function derives from selectionclick* and selectionqueue* and mirrors selectionpolygon

hs = [ handles.selectionmode ...
       handles.selectionsurface ...
       handles.selectionsurfaceadd ...
       handles.selectionsurfacedelete ...
       handles.selectionsurfaceoutline ...
       handles.selectiondraw ...
       handles.selectionhide ...
       handles.selectionclear ...
       handles.selectionunclear ...
       handles.selectionpolygon ...
       handles.selectionfunction ...
       handles.selectionqueue ...
     ];

if get(hObject,'Value')
  set(hs,'Enable','off');
  return;
end

if isempty(handles.cpqueue)
  set(hs,'Enable','on');
  selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button
  return;
end

if length(handles.cpqueue)<2
  fprintf(1,'\nerror: must define at least two points for line-based selection.\n');
  handles.cpqueue = {};
  set(hs,'Enable','on');
  selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button
  guidata(handles.output, handles);
  return;
end

% ok, determine the clicked vertices
vidxcollect = zeros(1,length(handles.cpqueue));
for p=1:length(handles.cpqueue)
  [dummy,dummy,vidx] = selectionclick_Callback_helper_helper(handles.cpqueue{p});
  if isempty(vidx)
    fprintf(1,'\nerror: did not find an intersection point for one or more of the points clicked.\n');
    handles.cpqueue = {};
    set(hs,'Enable','on');
    selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button
    guidata(handles.output, handles);
    return;
  end
  vidxcollect(p) = vidx;
end
handles.cpqueue = {};

% connect up those vertices
goodv = viewsurfacedata_connectpts(vidxcollect);
%keyboard;  % WE WANT TO EXPORT THIS

% pass off to helper (it saves handles)
selection_helper(handles,eventdata,goodv);

% lastly (good thing this doesn't modify handles)
set(hs,'Enable','on');
selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button

%%%%%%%%%%%%%%%

function selectionpolygon_Callback(hObject, eventdata, handles)

% this function derives from selectionclick* and selectionqueue* and mirrors selectionline

global VS_TXYZ VS_TNORMALS PIP_VERTICES VS_TISOLATED;

hs = [ handles.selectionmode ...
       handles.selectionsurface ...
       handles.selectionsurfaceadd ...
       handles.selectionsurfacedelete ...
       handles.selectionsurfaceoutline ...
       handles.selectiondraw ...
       handles.selectionhide ...
       handles.selectionclear ...
       handles.selectionunclear ...
       handles.selectionline ...
       handles.selectionfunction ...
       handles.selectionqueue ...
       handles.camerareset ...
       handles.camerapreset ...
       handles.camerapresetsave ...
       handles.camerapresetdelete ...
       handles.camerapresetrename ...
       handles.rotup ...
       handles.rotleft ...
       handles.rollccw ...
       handles.scrollup ...
       handles.scrollleft ...
       handles.zoom ...
       handles.viewangle ...
       handles.projection ...
       handles.xdir
     ];

if get(hObject,'Value')
  set(hs,'Enable','off');
  return;
end

if isempty(handles.cpqueue)
  set(hs,'Enable','on');
  selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button
  return;
end

if length(handles.cpqueue)<3
  fprintf(1,'\nerror: must define at least three points for polygon-based selection.\n');
  handles.cpqueue = {};
  set(hs,'Enable','on');
  selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button
  guidata(handles.output, handles);
  return;
end

% collect queued points
pts = [];
for p=1:length(handles.cpqueue)
  elt = handles.cpqueue{p};
  if p==1
    proj = elt{2};
    pos = elt{3};
    target = elt{4};
  else
    assert(isequal(elt{2},proj));
    assert(isequal(elt{3},pos));
    assert(isequal(elt{4},target));
  end
  if ~isscalar(elt{1})
    pts = [pts [elt{1}(2,:) 1]'];
  else
    pts = [pts VS_TXYZ(:,elt{1})];  % hm..., how does this case occur?
  end
end
handles.cpqueue = {};





% TODO: the following should probably be abstracted into a function (so as to get out 2D coordinates)

% in the initial/simplest case, v is [0 0 x] which means the vector-to-the-camera is right along the positive z-axis
v = pos-target;

% we want to translate the camera target to the origin and then rotate the vector-to-the-camera to coincide with the
% positive z-axis.
transform = xyzrotatetoz(v)*xyztranslate(-target);

% ok, now transform the vertices and the polygon
vrot = transform*VS_TXYZ;
ptsrot = transform*pts;

% ok, now disregard the z-coordinate, and do it!!!!
PIP_VERTICES = vrot(1:2,:)';
inout = pointinpolygon(ptsrot(1:2,:)');

% now we know which vertices are inside the polygon.  yay.
goodv = find(inout);

% filter vertices on the normals and on isolatedness
goodvdot = dot(repmat(v',[1 length(goodv)]),VS_TNORMALS(1:3,goodv),1);
goodv = goodv(goodvdot > 0 & ~VS_TISOLATED(goodv));





% pass off to helper (it saves handles)
selection_helper(handles,eventdata,goodv);

% lastly (good thing this doesn't modify handles)
set(hs,'Enable','on');
selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button

%%%%%%%%%%%%%%%

function selection_helper(handles,eventdata,goodv)

% <goodv> is a vector of vertex indices.  these are
%   guaranteed not to include any isolated vertices.
%
% we process <goodv> for selection-related purposes,
% including potential redrawing and selection-triggered
% function calling.  we save handles.

% if we have voxel information, calculate dindices,X,Y,Z
if handles.epion
  % which voxels contain at least one of the vertices
  dindices = unique(viewsurfacedata_lookup(-1,goodv));
  % transform
  [X,Y,Z] = viewsurfacedata_indextovoxel(dindices);
end

sval = handles.ssurf(get(handles.selectionsurface,'Value'));
switch handles.surfacerecord{sval}.selectionmode
case 1
  switch get(handles.selectiondraw,'String')
  case 'off'
    fprintf(1,'\nselection left unchanged since draw mode is ''off''.\n');
  case 'draw'
    prev = count(handles.epiclicked{sval}(dindices));
    if ~all(handles.epiclicked{sval}(dindices))
      handles.epiclicked{sval}(dindices) = 1;
      handles.clickedchanged{sval} = 1;
      handles.draworderchanged = 1;
    end
    cur = count(handles.epiclicked{sval}(dindices));
    fprintf(1,'\n%d voxels selected; %d voxels were already selected\n',cur-prev,prev);
  case 'erase'
    prev = count(~handles.epiclicked{sval}(dindices));
    if ~all(~handles.epiclicked{sval}(dindices))
      handles.epiclicked{sval}(dindices) = 0;
      handles.clickedchanged{sval} = 1;
      handles.draworderchanged = 1;
    end
    cur = count(~handles.epiclicked{sval}(dindices));
    fprintf(1,'\n%d voxels unselected; %d voxels were already unselected\n',cur-prev,prev);
  end
case 2
  switch get(handles.selectiondraw,'String')
  case 'off'
    fprintf(1,'\nselection left unchanged since draw mode is ''off''.\n');
  case 'draw'
    prev = count(handles.vclicked{sval}(goodv));
    if ~all(handles.vclicked{sval}(goodv))
      handles.vclicked{sval}(goodv) = 1;
      handles.clickedchanged{sval} = 1;
      handles.draworderchanged = 1;
    end
    cur = count(handles.vclicked{sval}(goodv));
    fprintf(1,'\n%d vertices selected; %d vertices were already selected\n',cur-prev,prev);
  case 'erase'
    prev = count(~handles.vclicked{sval}(goodv));
    if ~all(~handles.vclicked{sval}(goodv))
      handles.vclicked{sval}(goodv) = 0;
      handles.clickedchanged{sval} = 1;
      handles.draworderchanged = 1;
    end
    cur = count(~handles.vclicked{sval}(goodv));
    fprintf(1,'\n%d vertices unselected; %d vertices were already unselected\n',cur-prev,prev);
  end
end

% save/redraw
if isequal(get(handles.manual,'String'),'manual') || ...
   ~handles.clickedchanged{sval} && isequal(get(handles.selectionhide,'String'),'show')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

% apply function if necessary; this mirrors selectionclick* below
if get(handles.selectionfunction,'Value')
  % sanity check
  assert(handles.epion);
  % we do not need to call guidata here since we already saved above.
  for p=1:length(X)
    str = mat2str([X(p) Y(p) Z(p)]);
    fprintf(1,'applying selection-triggered function to voxel %s.\n',str);
    evalin('base',strrep(handles.func,'$VOXEL$',str));
  end
end

%%%%%%%%%%%%%%%

function selectionfunction_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%

function selectionqueue_Callback(hObject, eventdata, handles)

hs = [ handles.selectionmode ...
       handles.selectionsurface ...
       handles.selectionsurfaceadd ...
       handles.selectionsurfacedelete ...
       handles.selectionsurfaceoutline ...
       handles.selectiondraw ...
       handles.selectionhide ...
       handles.selectionclear ...
       handles.selectionunclear ...
       handles.selectionline ...
       handles.selectionpolygon ...
       handles.selectionfunction ...
     ];

if get(hObject,'Value')
  set(hs,'Enable','off');
else
  for p=1:length(handles.cpqueue)
    handles = selectionclick_Callback_helper(handles, handles.cpqueue{p});
  end
  handles.cpqueue = {};
  set(hs,'Enable','on');
  selectionsurface_Callback(handles.selectionsurface,eventdata,handles);  % for the delete button

  if isequal(get(handles.manual,'String'),'manual')
    guidata(handles.output, handles);
  else
    redraw_Callback(handles.redraw, eventdata, handles);
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SURFACE-RELATED

function surface_Callback(hObject, eventdata, handles)

% we do not save handles since there is no need to.

global VS_PDATA;

% which surface are we controlling?
surfval = get(hObject,'Value');

% set the colormap selector (this section should mirror stuff above)
% also, set the context menu specific stuff
if ismember(surfval,handles.rsurf)
  set(handles.colormap,'String',handles.cmaps);
  set(handles.visibility,'UIContextMenu',handles.visibilitycontext);
  set(handles.visibilitycmdot,'String','*');
else
  if ismember(surfval,handles.csurf)
    set(handles.colormap,'String',viewsurfacedata_constants('colornamesdouble'));
  else  % catch-all-else!
    set(handles.colormap,'String',viewsurfacedata_constants('colornames'));
  end
  set(handles.visibility,'UIContextMenu',[]);
  set(handles.visibilitycmdot,'String','');  % prefer this rather than visible setting due to werid mac os x11 redrawing problems
end

% set other things
set(handles.markers,'String',choose(handles.surfacerecord{surfval}.markers,'on','off'));
set(handles.visibility,'String',choose(handles.surfacerecord{surfval}.visibility,'on','off'));
set(handles.colormap,'Value',handles.surfacerecord{surfval}.colormap);
set(handles.alpha,'Value',handles.surfacerecord{surfval}.alpha);

% set other things that hinge...
if ismember(surfval,handles.rsurf) && ~isempty(VS_PDATA{surfval-1}) || ...
   ismember(surfval,[handles.csurf handles.psurf])
  set(handles.pvaluetext,'Enable','on');
  set(handles.pvaluereverse,'Enable','on');
  set(handles.pvaluereverse,'Value',handles.surfacerecord{surfval}.pvaluereverse);
  set(handles.pvalue,'Enable','on');
  set(handles.pvalue,'String',num2str(handles.surfacerecord{surfval}.pvalue));
else
  set(handles.pvaluetext,'Enable','off');
  set(handles.pvaluereverse,'Enable','off');
  set(handles.pvaluereverse,'Value',1);
  set(handles.pvalue,'Enable','off');
  set(handles.pvalue,'String','(n/a)');
end
if ismember(surfval,handles.vsurf)
  set(handles.edgestext,'Enable','off');
  set(handles.edges,'Enable','off');
  set(handles.edges,'String','off');
else
  set(handles.edgestext,'Enable','on');
  set(handles.edges,'Enable','on');
  set(handles.edges,'String',choose(handles.surfacerecord{surfval}.edges,'on','off'));
end
if ismember(surfval,handles.psurf)
  set(handles.offsettext,'Enable','off');
  set(handles.offset,'Enable','off');
  set(handles.offset,'String','n/a');
else
  set(handles.offsettext,'Enable','on');
  set(handles.offset,'Enable','on');
  set(handles.offset,'String',num2str(handles.surfacerecord{surfval}.offset));
end
if ismember(surfval,[handles.bsurf handles.psurf handles.vsurf])
  set(handles.boundarytext,'Enable','off');
  set(handles.boundary,'Enable','off');
  set(handles.boundary,'String',{'n/a'});
  set(handles.boundary,'Value',1);
else
  set(handles.boundarytext,'Enable','on');
  set(handles.boundary,'Enable','on');
  if ismember(surfval,[handles.ssurf])
    set(handles.boundary,'String',{'expand' 'restrict' 'outline'});
  else
    set(handles.boundary,'String',{'expand' 'restrict'});
  end
  set(handles.boundary,'Value',handles.surfacerecord{surfval}.boundary);
end

%%%%%%%%%%%%%%%

function edges_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
handles.surfacerecord{surfval}.edges = ~handles.surfacerecord{surfval}.edges;
set(hObject,'String',choose(handles.surfacerecord{surfval}.edges,'on','off'));

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function markers_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
handles.surfacerecord{surfval}.markers = ~handles.surfacerecord{surfval}.markers;
set(hObject,'String',choose(handles.surfacerecord{surfval}.markers,'on','off'));

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function visibility_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
if surfval==1  % force brain surface always visible
  set(hObject,'Value',2);
  return;
end
handles.surfacerecord{surfval}.visibility = ~handles.surfacerecord{surfval}.visibility;
set(hObject,'String',choose(handles.surfacerecord{surfval}.visibility,'on','off'));
handles.draworderchanged = 1;

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function colormap_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
handles.surfacerecord{surfval}.colormap = get(hObject,'Value');

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function alpha_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
  % force brain surface alpha and partial volume lines alpha to be 1
if ismember(surfval,[handles.bsurf handles.psurf])
  set(hObject,'Value',1);
  return;
end
handles.surfacerecord{surfval}.alpha = get(hObject,'Value');

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function pvaluereverse_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
assert(ismember(surfval,[handles.rsurf handles.csurf handles.psurf]));  % should not exist for brain or selection surface or boundaries surface
val = get(hObject,'Value');
if handles.surfacerecord{surfval}.pvaluereverse ~= val
  handles.surfacerecord{surfval}.pvaluereverse = val;
  handles.surfacerecord{surfval}.pvaluechanged = 1;
  % don't need to set draworderchanged if this is the partial volume case
  if ~ismember(surfval,handles.psurf)
    handles.draworderchanged = 1;
  end
end

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function pvalue_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
assert(ismember(surfval,[handles.rsurf handles.csurf handles.psurf]));  % should not exist for brain or selection surface or boundaries surface
val = str2double(get(hObject,'String'));
if isnan(val)
  set(hObject,'String',num2str(handles.surfacerecord{surfval}.pvalue));
  return;
end
if handles.surfacerecord{surfval}.pvalue ~= val
  handles.surfacerecord{surfval}.pvalue = val;
  handles.surfacerecord{surfval}.pvaluechanged = 1;  % note that this has no effect in the partial volume case
  % don't need to set draworderchanged if this is the partial volume case
  if ~ismember(surfval,handles.psurf)
    handles.draworderchanged = 1;
  end
end

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function offset_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
val = str2double(get(hObject,'String'));
if isnan(val) || val<0
  set(hObject,'String',num2str(handles.surfacerecord{surfval}.offset));
  return;
end
if surfval==1  % force brain surface to have zero offset
  set(hObject,'String','0');
  return;
end
if handles.surfacerecord{surfval}.offset ~= val
  handles.surfacerecord{surfval}.offset = val;
  handles.surfacerecord{surfval}.offsetchanged = 1;
  handles.draworderchanged = 1;
end

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%

function boundary_Callback(hObject, eventdata, handles)

surfval = get(handles.surface,'Value');
assert(ismember(surfval,[handles.rsurf handles.ssurf handles.csurf]));
val = get(hObject,'Value');
if handles.surfacerecord{surfval}.boundary ~= val
  % if we switched from or to the special outline boundary case, then
  % we may need to recalc the lookupbelow stuff.  so, set draworderchanged to 1.
  if handles.surfacerecord{surfval}.boundary==3 || val==3
    handles.draworderchanged = 1;
  end
  handles.surfacerecord{surfval}.boundary = val;
  handles.surfacerecord{surfval}.boundarychanged = 1;
end

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RENDER-RELATED

function redraw_Callback(hObject, eventdata, handles, replaceindices)

% replaceindices is optional.
% note that we save handles.

global VS_CMAPNUM VS_CMAPTYPE VS_SELECTION VS_BEGINVALUE VS_ENDVALUE;

% this is very important.
% we may get passed handles data that has not been saved yet.
% if any portion of this routine relies on the global VS_GUI
% to get handles, that data would be obsolete if we didn't save
% handles here.  so let's do it just for peace of mind.
guidata(handles.output, handles);

% get constants
normalshift = viewsurfacedata_constants('normalshift');

% deal with input
if ~exist('replaceindices','var')
  replaceindices = [];
end

% set status indicator
set(handles.status,'String','status: rendering...'); drawnow;

% deal with render window background
set(handles.fig,'Color',handles.bgcolors{get(handles.lightbackground,'Value')});

% deal with pts
switch get(handles.selectionhide,'String')
case 'hide'
  set(handles.ptclick,'Visible','on');
  set(handles.ptclick,'XData',handles.ptclickdata(1),'YData',handles.ptclickdata(2),'ZData',handles.ptclickdata(3));
%  set(handles.pts(2),'XData',handles.ptsdata(2,1),'YData',handles.ptsdata(2,2),'ZData',handles.ptsdata(2,3));
%  set(handles.pts(3),'XData',handles.ptsdata(3:4,1),'YData',handles.ptsdata(3:4,2),'ZData',handles.ptsdata(3:4,3));
%  set(handles.pts(4),'XData',handles.ptsdata(4,1),'YData',handles.ptsdata(4,2),'ZData',handles.ptsdata(4,3));
case 'show'
  set(handles.ptclick,'Visible','off');
end

% deal with voxel boundaries surface
if handles.epion && handles.surfacerecord{handles.vsurf(1)}.visibility
  % ok, looks like the user really wants to see the intersection surface.
  % if the surface has not been drawn yet or if the offset has been
  % changed, we have to draw from scratch.
  if isempty(get(handles.surfs(handles.vsurf(1)),'Vertices')) || ...
     handles.surfacerecord{handles.vsurf(1)}.offsetchanged
    viewsurfacedata_drawboundaries(handles.surfs(handles.vsurf(1)),1,handles.surfacerecord{handles.vsurf(1)}.offset*normalshift);
    viewsurfacedata_drawboundaries(handles.surfs(handles.vsurf(2)),2,handles.surfacerecord{handles.vsurf(1)}.offset*normalshift);
    viewsurfacedata_drawboundaries(handles.surfs(handles.vsurf(3)),3,handles.surfacerecord{handles.vsurf(1)}.offset*normalshift);
  end
  % otherwise, we don't have to do anything!
end

% deal with replacedata
for p=replaceindices
  viewsurfacedata_replacedata(handles.surfs(handles.rsurf(p)),p);
end

% recalculate handles.draworder if necessary
if handles.draworderchanged
  % ok, what is the vector of things to order?
  origorder = handles.layerorder;
  % ok, filter out the ones that aren't visible
  draworder = [];
  for p=origorder
    if handles.surfacerecord{p}.visibility
      draworder = [draworder p];
    end
  end
  % for those, use offset to reorder
  offsetorder = [];
  for p=1:length(draworder)
    offsetorder(p) = handles.surfacerecord{draworder(p)}.offset;
  end
  [offsetorder,idx] = sort(offsetorder);
  draworder = draworder(idx);
  % break ties using layer order
  p = 1;
  while 1
    % in this case we're done
    if p>=length(offsetorder)
      break;
    end
    % look for repeats
    temp = offsetorder(p+1:end) == offsetorder(p);
    if any(temp)
      % repeat found
      extra = lastel(find(temp));
      same = p:p+extra;
      sameidx = findorder(draworder(same),handles.layerorder);
      % mangle the repeats to be in the right order
      draworder(same) = subscript(draworder(same),sameidx);
      % increment
      p = p+extra+1;
    else
      % no repeat found
      p = p+1;
      continue;
    end
  end
  % ok, we have the draworder now!
  handles.draworder = draworder;
end

% always recalc draworderp and draworderpdir
draworderp = zeros(1,length(handles.draworder));
draworderpdir = zeros(1,length(handles.draworder));
for p=1:length(handles.draworder)
  draworderp(p) = handles.surfacerecord{handles.draworder(p)}.pvalue;
  draworderpdir(p) = handles.surfacerecord{handles.draworder(p)}.pvaluereverse;
end

% we need this in calls to _drawsurface.  the idea is that VS_SELECTION is
% always up-to-date (but modulo redrawing!) and indicates which vertices are
% in the selection surfaces.  
for sval=handles.ssurf
  if handles.clickedchanged{sval}
    switch handles.surfacerecord{sval}.selectionmode
    case 1
      [dummy,temp] = viewsurfacedata_lookup(-1,-1,handles.epiclicked{sval});
      VS_SELECTION{sval} = temp;
    case 2
      VS_SELECTION{sval} = find(handles.vclicked{sval});
    end
  end
end

% deal with regular, selection, and curvature surfaces
for p=1:length(handles.draworder)
  sval = handles.draworder(p);
  % regular surface case
  if ismember(sval,handles.rsurf) && (handles.draworderchanged || handles.surfacerecord{sval}.pvaluechanged || handles.surfacerecord{sval}.boundarychanged)
    handles.surfacerecord{sval}.pvaluechanged = 0;
    handles.surfacerecord{sval}.boundarychanged = 0;
    viewsurfacedata_drawsurface(2,handles.surfs(sval),sval-1, ...
      handles.surfacerecord{sval}.pvalue,handles.surfacerecord{sval}.pvaluereverse, ...
      handles.draworder(1:p-1),draworderp(1:p-1),draworderpdir(1:p-1), ...
      handles.surfacerecord{sval}.boundary);
    handles.surfacerecord{sval}.offsetchanged = 1;  % to ensure we get the correct offset
  end
  % selection surface case
  if ismember(sval,handles.ssurf) && (handles.draworderchanged || handles.clickedchanged{sval} || handles.surfacerecord{sval}.boundarychanged)
    handles.clickedchanged{sval} = 0;
    handles.surfacerecord{sval}.boundarychanged = 0;
    viewsurfacedata_drawsurface(3,handles.surfs(sval),sval-handles.ssurf(1)+1,VS_SELECTION{sval}, ...
    handles.draworder(1:p-1),draworderp(1:p-1),draworderpdir(1:p-1), ...
    handles.surfacerecord{sval}.boundary);
    handles.surfacerecord{sval}.offsetchanged = 1;  % to ensure we get the correct offset
  end
  % curvature surface case
  if ismember(sval,handles.csurf) && (handles.draworderchanged || handles.surfacerecord{sval}.pvaluechanged || handles.surfacerecord{sval}.boundarychanged)
    handles.surfacerecord{sval}.pvaluechanged = 0;
    handles.surfacerecord{sval}.boundarychanged = 0;
    viewsurfacedata_drawsurface(4,handles.surfs(sval), ...
      handles.surfacerecord{sval}.pvalue,handles.surfacerecord{sval}.pvaluereverse, ...
      handles.draworder(1:p-1),draworderp(1:p-1),draworderpdir(1:p-1), ...
      handles.surfacerecord{sval}.boundary);
    handles.surfacerecord{sval}.offsetchanged = 1;  % to ensure we get the correct offset
  end
end

% we are done with draworder stuff, so set flag off
handles.draworderchanged = 0;

% deal with brain, regular, selection, and curvature surfaces (the voxel boundaries surface deals with offset on its own).
% note that this section needs to come after any calls to _drawsurface so that the proper offset can be applied.
for p=[handles.bsurf handles.rsurf handles.ssurf handles.csurf]
  if handles.surfacerecord{p}.offsetchanged
    handles.surfacerecord{p}.offsetchanged = 0;
    if ~isempty(get(handles.surfs(p),'Vertices'))  % explicitly avoid weird degenerate cases.  HACKy.
      ud = get(handles.surfs(p),'UserData');
      set(handles.surfs(p),'Vertices', ...
          ud.origvertices + ...
          (handles.surfacerecord{p}.offset*normalshift)*get(handles.surfs(p),'VertexNormals'));
    end
  end
end

% deal with camera stuff
cpv = get(handles.camerapreset,'Value');
% if camera is to be reset or if we are hacking a preset in with *importcamerapreset or if some camera preset was selected
if handles.cameraresettoorig || ~isempty(handles.importcamerapreset) || cpv~=1
  if handles.cameraresettoorig
    preset = handles.cameraorig;
    handles.cameraresettoorig = 0;
  elseif ~isempty(handles.importcamerapreset)
    preset = handles.importcamerapreset;
    handles.importcamerapreset = [];
  else
    preset = handles.settings.camerapresets{cpv-1,2};
  end
  % let's deal with xdir FIRST to avoid order problems
  if length(preset) >= 6
    set(handles.xdir,'Value',choose(isequal(preset{6},'normal'),1,2));  % a bit weirdly placed?
  else
    set(handles.xdir,'Value',1);  % assume normal for backwards compatibility
  end
  set(handles.ax,'XDir',choose(get(handles.xdir,'Value')==1,'normal','reverse'));
  % ok, continue
  campos(handles.ax,preset{1});
  camproj(handles.ax,preset{2});
  camtarget(handles.ax,preset{3});
  camup(handles.ax,preset{4});
  handles.viewangleval = preset{5};
  if handles.viewangleval < 5
    fprintf(1,'\nwarning: the viewing angle loaded from the preset is less than 2 degrees.  this may cause weird rendering/clicking/etc. behavior!\n');
  end
  camva(handles.ax,handles.viewangleval);
  set(handles.projection,'Value',choose(isequal(preset{2},'perspective'),1,2));  % a bit weirdly placed...
% ok, normal case
else
  % let's deal with xdir FIRST to avoid order problems
  set(handles.ax,'XDir',choose(get(handles.xdir,'Value')==1,'normal','reverse'));
  camorbit(handles.ax,0,get(handles.rotup,'Value'),'camera');
  camorbit(handles.ax,-get(handles.rotleft,'Value'),0,'camera');
  camroll(handles.ax,-get(handles.rollccw,'Value'));
  camdolly(handles.ax,0,-get(handles.scrollup,'Value'),0);
  camdolly(handles.ax,get(handles.scrollleft,'Value'),0,0);
  campos(handles.ax,moveto(camtarget(handles.ax),campos(handles.ax),1/str2double(get(handles.zoom,'String'))));
  handles.viewangleval = str2double(get(handles.viewangle,'String'));
  camva(handles.ax,handles.viewangleval);
  camproj(handles.ax,subscript(get(handles.projection,'String'),get(handles.projection,'Value'),1));
end
lighttype = subscript(get(handles.lighttype,'String'),get(handles.lighttype,'Value'),1);
camlight(handles.lt,subscript(get(handles.lightpos,'String'),get(handles.lightpos,'Value'),1));

% deal with partial-volume voxels (needs to come after any camera changes!)
% this is totally separate from the handling of the .surfs stuff.
% this recalcs and recreates the lines even when the camera view doesn't change, oh well
if handles.epion
  if handles.surfacerecord{handles.psurf}.visibility
    % ok, looks like the user really wants to see the partial volume stuff.
    delete(handles.pvv);
    if isequal(camproj(handles.ax),'perspective')
      handles.pvv = ...
        viewsurfacedata_drawpartialvolumes(handles.fig,handles.ax, ...
          handles.surfacerecord{handles.psurf}.pvalue,handles.surfacerecord{handles.psurf}.pvaluereverse);
      set(handles.pvv,'Marker',choose(handles.surfacerecord{handles.psurf}.markers,'.','none'));
      set(handles.pvv,'Color',handles.pcolors{handles.surfacerecord{handles.psurf}.colormap});
      set(handles.pvv,'LineStyle',choose(handles.surfacerecord{handles.psurf}.edges,'-','none'));
    else
      handles.pvv = [];
      fprintf(1,'\nwarning: partial-volume voxels not drawn since the projection type is not perspective.\n');
    end
  else
    set(handles.pvv,'Visible','off');
  end
end

% deal with miscellaneous surface stuff (not including partial volume voxels)
for p=[handles.bsurf handles.rsurf handles.ssurf handles.csurf handles.vsurf];
  if ismember(p,handles.vsurf)
    sr = handles.surfacerecord{handles.vsurf(1)};   % hack
  else
    sr = handles.surfacerecord{p};
  end
  
  % this is a special case for the outline mode of the selection surfaces
  % and for the voxel boundaries surface.  awkward!
  if sr.boundary==3 || sr.edges==-1
    switch sr.edges
    case 0
      set(handles.surfs(p),'EdgeColor','none','EdgeAlpha',0);
    case {1 -1}
      if sr.boundary==3
        color = handles.scolors{sr.colormap};
      else
        color = handles.icolors{sr.colormap};
      end
      set(handles.surfs(p),'EdgeColor',color,'EdgeAlpha','flat');
      temp = get(handles.surfs(p),'FaceVertexAlphaData');
      if ~isempty(temp)
        ratio = sr.alpha/temp(firstel(find(temp~=0)));
          % note that FaceVertexAlphaData should be set even if ratio is 1 because MATLAB is quirky
        set(handles.surfs(p),'FaceVertexAlphaData',temp*ratio);
      end
    end
  else
    switch sr.edges
    case 0
      set(handles.surfs(p),'EdgeColor','none','EdgeAlpha',0);
    case 1
      set(handles.surfs(p),'EdgeColor','black','EdgeAlpha',handles.edgealphav);
    end
  end
  %  what about 'EdgeAlpha','interp'  ??
  
  set(handles.surfs(p),'Marker',choose(sr.markers,'.','none'));
  set(handles.surfs(p),'FaceLighting',lighttype);  % this sets for the intersection and outline-selection surfaces, but it doesn't matter
  set(handles.surfs(p),'Visible',choose(sr.visibility,'on','off'));
  set(handles.surfs(p),'FaceAlpha',sr.alpha);  % this sets for the intersection and outline-selection surfaces, but it doesn't matter
end

% deal with colormap
temp = [];
for p=fliplr(handles.ssurf)
  temp = [temp; handles.scolors{handles.surfacerecord{p}.colormap}];
end
temp = [temp; handles.bcolors{handles.surfacerecord{handles.bsurf}.colormap}];
if handles.curvon
  temp = [temp; handles.ccolors{handles.surfacerecord{handles.csurf}.colormap}];
else
  temp = [temp; 0 0 0; 0 0 0];
end
if handles.epion
  temp = [temp; handles.pcolors{handles.surfacerecord{handles.psurf}.colormap}];
else
  temp = [temp; 0 0 0];
end
for p=handles.rsurf
  str = [handles.cmaps{handles.surfacerecord{p}.colormap},'(',num2str(VS_CMAPNUM{p-1}),')'];
  tempcmap = [];
  try
    tempcmap = feval(handles.cmaps{handles.surfacerecord{p}.colormap},VS_CMAPNUM{p-1});
  catch
    fprintf(1,['\nthere was an error in evaluating "',str,'", so we are using a gray colormap instead.\n']);
    tempcmap = gray(VS_CMAPNUM{p-1});
  end
  if ~(size(tempcmap,1)==VS_CMAPNUM{p-1} && size(tempcmap,2)==3 && ndims(tempcmap)==2)
    fprintf(1,['\nthe result of evaluating "',str,'" has unexpected dimensions, so we are using a gray colormap instead.\n']);
    tempcmap = gray(VS_CMAPNUM{p-1});
  end
  if VS_CMAPTYPE{p-1}==1  % add first to end if circular case
    tempcmap = [tempcmap; tempcmap(1,:)];
  end
  % insert lower and upper paddings
  temp = [temp; tempcmap(1,:); tempcmap; tempcmap(end,:)];
end
colormap(handles.ax,temp);

% as the final thing before rendering (seems to trigger a redraw), change layer order if necessary
if handles.layerorderchanged  % this currently cannot be the case, but it works.
  handles.layerorderchanged = 0;
  objs = get(handles.ax,'Children');
  surfs = handles.surfs(handles.layerorder);
  objs(ismember(objs,surfs)) = fliplr(surfs)';  % weird reversal, then horizontal->vertical
  set(handles.ax,'Children',objs);
end

% deal with axis limits (we set this initially, but when selection surfaces get added / deleted,
% the beginvalue may change.  so we set here each time we redraw, oh well.)
assert(VS_BEGINVALUE ~= VS_ENDVALUE);
set(handles.ax,'CLim',[VS_BEGINVALUE VS_ENDVALUE]);

% render
drawnow;

% handle status
set(handles.status,'String','status: ready'); drawnow;

% handles polygon
  % if camera view is orthographic and we're not in the middle of a queue
if isequal(camproj(handles.ax),'orthographic') && ~get(handles.selectionqueue,'Value')
  set(handles.selectionpolygon,'Enable','on');
else
  set(handles.selectionpolygon,'Enable','off');
end

% lastly, reset the appropriate controls
set(handles.rotup,'Value',0);
set(handles.rotleft,'Value',0);
set(handles.rollccw,'Value',0);
set(handles.scrollup,'Value',0);
set(handles.scrollleft,'Value',0);
set(handles.zoom,'String','1');
set(handles.viewangle,'String',num2str(handles.viewangleval));
% note that projection/xdir is set earlier (if it is necessary)
set(handles.camerapreset,'Value',1);
camerapreset_update(handles);

% save
guidata(handles.output, handles);

%%%%%%%%%%%%%%%

function manual_Callback(hObject, eventdata, handles)

switch get(hObject,'String')
case 'manual'
  set(hObject,'String','auto');
case 'auto'
  set(hObject,'String','manual');
end

%%%%%%%%%%%%%%%

function snapshot_Callback(hObject, eventdata, handles)
filename = writesnapshot(handles.fig,[],[],[],[],handles.settings.snapshotmethod);
fprintf(1,['\nsnapshot successfully written to ',filename,'.\n']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATED

function selectionclick_Callback(hObject, eventdata, handles, vertex)

global VS_TISOLATED;

% <vertex> is optional.  if supplied, it must be the case that it is 
% a non-isolated vertex, and we use that information instead of
% CurrentPoint.

if ~exist('vertex','var')
  elt = { get(handles.ax,'CurrentPoint') camproj(handles.ax) campos(handles.ax) camtarget(handles.ax) };
else
  assert(~VS_TISOLATED(vertex));
  elt = { vertex camproj(handles.ax) campos(handles.ax) camtarget(handles.ax) };
end

% if we are in queue mode
if get(handles.selectionline,'Value') || get(handles.selectionpolygon,'Value') || get(handles.selectionqueue,'Value')
  % just add to queue and save; never redraw
  handles.cpqueue{end+1} = elt;
  guidata(handles.output, handles);
else
  % otherwise, handle right away
  handles = selectionclick_Callback_helper(handles, elt);
  % ok, if we are in manual mode, definitely don't need to redraw.
  % if we are in auto mode, only draw if clickedchanged or if
  % the pinpoints are being shown.
  if isequal(get(handles.manual,'String'),'manual') || ...
     ~anycell(handles.clickedchanged(handles.ssurf)) && isequal(get(handles.selectionhide,'String'),'show')
    guidata(handles.output, handles);
  else
    redraw_Callback(handles.redraw, eventdata, handles);
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% UTILITIES

function camerapreset_update(handles)

% based on the status of camerapreset, enable and disable as needed.
% should be called after every possible change to the drop-down selection.

hs1 = [handles.rotup ...
       handles.rotuptext ...
       handles.rotleft ...
       handles.rotlefttext ...
       handles.rollccw ...
       handles.rollccwtext ...
       handles.scrollup ...
       handles.scrolluptext ...
       handles.scrollleft ...
       handles.scrolllefttext ...
       handles.zoom ...
       handles.zoomtext ...
       handles.viewangle ...
       handles.viewangletext ...
       handles.projection ...
       handles.projectiontext ...
       handles.xdir ...
       handles.xdirtext];
hs2 = [ handles.camerapresetdelete ...
        handles.camerapresetrename ];
hs3 = [ handles.camerapresetsave ];

if get(handles.camerapreset,'Value')==1
  set(hs1,'Enable','on');
  set(hs2,'Enable','off');
  set(hs3,'Enable','on');
else
  set(hs1,'Enable','off');
  set(hs2,'Enable','on');
  set(hs3,'Enable','on');
end

%%%%%%%%%%%%%%%

function camerapreset_reset(handles)

% based on handles.settings.camerapresets, re-fill the
% drop-down and leave it on the first setting (blank).
% we automatically call camerapreset_update, too.

temp = { '' };
for p=1:size(handles.settings.camerapresets,1)
  temp = [temp { handles.settings.camerapresets{p,1} }];
end
set(handles.camerapreset,'String',temp);
set(handles.camerapreset,'Value',1);
camerapreset_update(handles);

%%%%%%%%%%%%%%%

function handles = selectionclick_Callback_helper(handles, cpqueueelt)

% <cpqueueelt> is like { cp|vidx camproj campos camtarget }
%   in the case that vidx is used, it must be a non-isolated vertex.
%
% note that we do not redraw (not even if in auto redraw mode)
% you must redraw for me if necessary.
%
% for speed, we do not save handles.  we return it.
% you must save it yourself.

global VS_R VS_RPTR VS_PDATA VS_TXYZ VS_TNORMALS VS_TCOLORS;

% find intersection point
[ip,cp,vidx] = selectionclick_Callback_helper_helper(cpqueueelt);
if isempty(ip)
  fprintf(1,'\nerror: did not find an intersection point!\n');
  return;
end

% feedback
normal = VS_TNORMALS(1:3,vidx)';
%a1 = [ip; ip + 10*normal];
%a2 = [cp; cp + 10*normal];
%handles.ptclickdata = [a1; a2];
handles.ptclickdata = cp + .01*normal;  % FIXME
% report vertex location
fprintf(1,'\nsurface coordinates of clicked vertex (%d): %s.\n',vidx,mat2str(VS_TXYZ(1:3,vidx)',6));
% for each surface, report
didx = NaN*zeros(1,length(handles.rsurf));  % this will hold the data index (if voxel-specified) for each data set
for p=1:length(handles.rsurf)
  [dindices,vindices] = viewsurfacedata_access(p);
  temp = ismember(vindices,vidx);
  if ~any(temp)
    fprintf(1,'surface %d: vertex does not lie in valid data region.\n',p);
  else
    didx(p) = dindices(temp);
    assert(length(didx(p))==1);  % sanity check
    % report surface values
    fprintf(1,'surface %d: vertex has value %f.\n', ...
            p,viewsurfacedata_backtransform(p,didx(p)));
    if ~isempty(VS_PDATA{p})
      fprintf(1,'surface %d: vertex has p-value %f.\n', ...
              p,VS_PDATA{p}(didx(p)));
    end
  end
end
% if we have voxel information and if the clicked vertex has a corresponding data index,
% then lookup X,Y,Z
if handles.epion && ~isnan(didx(1))
  [X,Y,Z] = viewsurfacedata_indextovoxel(didx(1));
end
% do it
for val=1:length(handles.ssurf)
  sval = handles.ssurf(val);
  switch handles.surfacerecord{sval}.selectionmode
  case 1
    if ~isnan(didx(1))
      % if draw mode is off or if this is not the current selection surface
      if isequal(get(handles.selectiondraw,'String'),'off') || get(handles.selectionsurface,'Value')~=val
        if handles.epiclicked{sval}(didx(1))
          fprintf(1,['selection surface %d: ',mat2str([X Y Z]),' (selected)\n'],val);
        else
          fprintf(1,['selection surface %d: ',mat2str([X Y Z]),' (unselected)\n'],val);
        end
      else
        switch get(handles.selectiondraw,'String')
        case 'draw'
          if handles.epiclicked{sval}(didx(1))
            fprintf(1,['selection surface %d: ',mat2str([X Y Z]),' (already selected)\n'],val);
          else
            handles.epiclicked{sval}(didx(1)) = 1;
            handles.clickedchanged{sval} = 1;
            handles.draworderchanged = 1;
            fprintf(1,['selection surface %d: ',mat2str([X Y Z]),' selected\n'],val);
          end
        case 'erase'
          if ~handles.epiclicked{sval}(didx(1))
            fprintf(1,['selection surface %d: ',mat2str([X Y Z]),' (already unselected)\n'],val);
          else
            handles.epiclicked{sval}(didx(1)) = 0;
            handles.clickedchanged{sval} = 1;
            handles.draworderchanged = 1;
            fprintf(1,['selection surface %d: ',mat2str([X Y Z]),' unselected\n'],val);
          end
        end
      end
    end
  case 2
    % if draw mode is off or if this is not the current selection surface
    if isequal(get(handles.selectiondraw,'String'),'off') || get(handles.selectionsurface,'Value')~=val
      if handles.vclicked{sval}(vidx)
        fprintf(1,['selection surface %d: vertex ',num2str(vidx),' (selected)\n'],val);
      else
        fprintf(1,['selection surface %d: vertex ',num2str(vidx),' (unselected)\n'],val);
      end
    else
      switch get(handles.selectiondraw,'String')
      case 'draw'
        if handles.vclicked{sval}(vidx)
          fprintf(1,['selection surface %d: vertex ',num2str(vidx),' (already selected)\n'],val);
        else
          handles.vclicked{sval}(vidx) = 1;
          handles.clickedchanged{sval} = 1;
          handles.draworderchanged = 1;
          fprintf(1,['selection surface %d: vertex ',num2str(vidx),' selected\n'],val);
        end
      case 'erase'
        if ~handles.vclicked{sval}(vidx)
          fprintf(1,['selection surface %d: vertex ',num2str(vidx),' (already unselected)\n'],val);
        else
          handles.vclicked{sval}(vidx) = 0;
          handles.clickedchanged{sval} = 1;
          handles.draworderchanged = 1;
          fprintf(1,['selection surface %d: vertex ',num2str(vidx),' unselected\n'],val);
        end
      end
    end
  end
end
% for curvature surface, report
if handles.curvon
  switch VS_TCOLORS(vidx)
  case 0
    fprintf(1,'clicked vertex has convex curvature.\n');
  case 1
    fprintf(1,'clicked vertex has concave curvature.\n');
  end
end
% for partial-volume surface, report
if handles.epion && ~isnan(didx(1))
  % only report if we've already calculated it
  if isfield(VS_R{VS_RPTR{1}},'pvv')
    fprintf(1,'clicked voxel has %d within-surfaces.\n',VS_R{VS_RPTR{1}}.pvv(didx(1)));
  else
    fprintf(1,'clicked voxel has unknown within-surfaces, since that information has not yet been calculated yet.\n');
  end
end
% apply function if necessary
if handles.epion && ~isnan(didx(1)) && get(handles.selectionfunction,'Value')
  guidata(handles.output, handles);  % ok, we have to save handles for freshness. this may be a slowdown, but oh well.
  str = mat2str([X Y Z]);
  fprintf(1,'applying selection-triggered function to voxel %s.\n',str);
  evalin('base',strrep(handles.func,'$VOXEL$',str));
end

%%%%%%%%%%%%%%%

function [ip,cp,vidx] = selectionclick_Callback_helper_helper(cpqueueelt)

% <cpqueueelt> is like { cp|vidx camproj campos camtarget }
%   if vidx is used, it must be a non-isolated vertex.
%
% return <ip>, the point of intersection with a face (1 x 3)
% return <cp>, the vertex of the face closest to <ip>.
% return <vidx>, the vertex index.
%
% if there is no intersection, <ip>==<cp>==[].
%
% note that in the special case of <vidx>, then ip==cp.

global VS_TFACES VS_TXYZ VS_TISOLATED;

if isscalar(cpqueueelt{1})
  vidx = cpqueueelt{1};
  assert(~VS_TISOLATED(vidx));
  ip = VS_TXYZ(1:3,vidx)';
  cp = ip;
else
  if isequal(cpqueueelt{2},'perspective')
    endpt = cpqueueelt{1}(2,:);
    startpt = cpqueueelt{3};
  else
    endpt = cpqueueelt{1}(2,:);
    startpt = endpt + (cpqueueelt{3}-cpqueueelt{4});  % add (campos-camtarget)
  end
  [ip,cp,cpi] = intersectraytri(startpt,endpt,0);
  if ~isempty(cpi)
    vidx = VS_TFACES(cpi(1),cpi(2));
  else
    vidx = [];
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTEXT MENUS

function visibilitycontext_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%

function vcontextisolateon_Callback(hObject, eventdata, handles)
vcontext_helper(hObject, eventdata, handles, 's==scur');

%%%%%%%%%%%%%%%

function vcontextisolateoff_Callback(hObject, eventdata, handles)
vcontext_helper(hObject, eventdata, handles, 's~=scur');

%%%%%%%%%%%%%%%

function vcontextallon_Callback(hObject, eventdata, handles)
vcontext_helper(hObject, eventdata, handles, '1');

%%%%%%%%%%%%%%%

function vcontextalloff_Callback(hObject, eventdata, handles)
vcontext_helper(hObject, eventdata, handles, '0');

%%%%%%%%%%%%%%%

function vcontext_helper(hObject, eventdata, handles, str)

scur = get(handles.surface,'Value');
for p=1:length(handles.rsurf)
  s = handles.rsurf(p);
  handles.surfacerecord{s}.visibility = eval(str);
end
handles.draworderchanged = 1;

surface_Callback(handles.surface,[],handles);

if isequal(get(handles.manual,'String'),'manual')
  guidata(handles.output, handles);
else
  redraw_Callback(handles.redraw, eventdata, handles);
end
