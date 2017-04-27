function f = viewsurfacedata_toggle(visibility,wantreport)

% function f = viewsurfacedata_toggle(visibility)
%
% f = viewsurfacedata_toggle
%   returns a vector of elements like this:
%     0 means visibility is off
%     1 means visibility is on
%
% viewsurfacedata_toggle(visibility)
%   sets the visibility according to <visibility>, 
%     which is a vector of elements like this:
%       0 means set visibility off
%       1 means set visibility on
%       2 means toggle current visibility setting
%   the <visibility> vector must have length equal to the total
%   number of surfaces (as shown in the surface drop-down).
%   <visibility> can also be a scalar, in which case we automatically
%   use that value for all surfaces.
%
%   after we update the visibility settings, we immediately
%   force a redraw of the render window.
%   
%   if <visibility> is [], that is equivalent to the
%   first type of call.

global VS_GUI;

% deal with input
if ~exist('wantreport','var')
  wantreport = 1;
end

% init
f = [];

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

if exist('visibility','var') && ~isempty(visibility)
  % deal with input
  if numel(visibility)==1
    visibility = repmat(visibility,[1 length(handles.surfacerecord)]);
  end
  % check input
  if ~(isrowvector(visibility) && ...
       length(visibility)==length(handles.surfacerecord) && ...
       all(visibility==0 | visibility==1 | visibility==2))
    fprintf(1,'error: invalid format for <visibility>.\n');
    return;
  end
  
  for p=1:length(handles.surfacerecord)
    switch visibility(p)
    case {0 1}
      handles.surfacerecord{p}.visibility = visibility(p);
    case 2
      handles.surfacerecord{p}.visibility = ...
        choose(handles.surfacerecord{p}.visibility,0,1);
    end
  end
  handles.draworderchanged = 1;
  
  % update the surface gui stuff
  viewsurfacedata_gui('surface_Callback',handles.surface,[],handles);
  
  % force a redraw
  viewsurfacedata_gui('redraw_Callback',handles.redraw,[],handles);
  
  % report
  if wantreport
    fprintf(1,'visibility settings successfully changed.\n');
  end
else
  f = [];
  for p=1:length(handles.surfacerecord)
    f = [f handles.surfacerecord{p}.visibility];  % TODO: make into function?
  end

  % construct labels
  labels = get(handles.surface,'String');
  labels(:,2) = num2cell(f)';

  % report
  if wantreport
    fprintf(1,'the current surface visibility is as follows:\n');
    disp(labels);
  end
end
