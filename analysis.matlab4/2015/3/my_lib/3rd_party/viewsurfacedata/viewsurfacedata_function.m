function f = viewsurfacedata_function(str)

% function f = viewsurfacedata_function(str)
%
% f = viewsurfacedata_function
%   returns the current selection-triggered function string.
%   the default string is 'disp(mat2str($VOXEL$))' which simply causes
%   the voxel location to be echoed in the command window.
%
% viewsurfacedata_function(str)
%   sets the selection-triggered function string to <str>.  
%   if <str> is [], we set <str> to the default string.  
%   otherwise, <str> must be an expression that can be
%   evaluated in the base workspace after substituting
%   a voxel location (like [32 20 15]) for the 
%   substring '$VOXEL$'.  for example, a valid function
%   string is 'prod($VOXEL$)', which will cause the expression
%   prod([32 20 15]) to be evaluated in the base workspace
%   when voxel [32 20 15] is clicked.

global VS_GUI;

% init
f = [];

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

if exist('str','var')
  % deal with input
  if isempty(str)
    str = 'disp(mat2str($VOXEL$))';
  end
  % check
  if isempty(strfind(str,'$VOXEL$'))
    fprintf(1,'error: <str> must contain at least one occurrence of the string ''$VOXEL$''.\n');
    return;
  end
  % set
  handles.func = str;
  % save
  guidata(handles.output, handles);
  % report
  fprintf(1,'the selection-triggered function has been successfully changed.\n');
else
  f = handles.func;
end
