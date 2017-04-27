function viewsurfacedata_replace(replacedata,replaceindices)

% function viewsurfacedata_replace(replacedata,replaceindices)
%
% <replacedata> is like <data> in viewsurfacedata except
%   that case (2) is not allowed.
% <replaceindices> (optional) is a vector of <data>
%   positions which indicate the positions of the
%   datasets which <replacedata> is to replace.  
%   as expected, these positions are in respect to
%   the original call to viewsurfacedata.  if [] or
%   not supplied, default to 1:length(replacedata).
%
% based on <replaceindices>, replace the already-loaded
% <data> with <replacedata>, and immediately force a
% redraw of the render window.
%
% note that the <replacedata> matrix must have the same
% dimensions, orientation, and data range as the
% corresponding <data> matrix that is to be replaced.

global VS_DATA VS_CDRANGE VS_DRANGE VS_GUI;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% preliminary check
if get(handles.selectionqueue,'Value')
  fprintf(1,['warning: the selection queue mode is active.  when unqueued, ', ...
             'the subsequent selection-related events will access the latest ', ...
             'state of data, not necessarily the state of data ', ...
             'when the selections were queued.']);
end

% transform input
if ~iscell(replacedata)
  replacedata = {replacedata};
end

% deal with input
if ~exist('replaceindices') || isempty(replaceindices)
  replaceindices = 1:length(replacedata);
end

% sanity check
replaceindices = unique(replaceindices);
if ~all(ismember(replaceindices,1:length(VS_DATA)))
  fprintf(1,'error: elements of <replaceindices> must be an integer between 1 and the number of data surfaces.\n');
  return;
end
if length(replacedata)~=length(replaceindices)
  fprintf(1,'error: mismatch in the length of <replacedata> and the length of <replaceindices>.\n');
  return;
end
for p=1:length(replacedata)
  idx = replaceindices(p);
  if ~isequal(size(replacedata{p}),size(VS_DATA{idx}))
    fprintf(1,['error: dimensions of replacedata number ',num2str(p),' do not match the dimensions of ', ...
               'the data stored for surface number ',num2str(idx),'.\n']);
    return;
  end
end

% do it
for p=1:length(replacedata)
  idx = replaceindices(p);
    % this mirrors viewsurfacedata
  VS_DATA{idx} = normalizerange(replacedata{p},VS_CDRANGE{idx}(1),VS_CDRANGE{idx}(2),VS_DRANGE{idx}(1),VS_DRANGE{idx}(2),1);  % note that we chop!
end

% yep!
handles.draworderchanged = 1;

% force a redraw
viewsurfacedata_gui('redraw_Callback',handles.redraw,[],handles,replaceindices);

% report
fprintf(1,'data successfully replaced.\n');
