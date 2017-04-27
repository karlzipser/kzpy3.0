function [datacollect,pdatacollect] = viewsurfacedata_exportsurfaces(filtervisible,filterpvalue,exportindices)

% function [datacollect,pdatacollect] = viewsurfacedata_exportsurfaces(filtervisible,filterpvalue,exportindices)
%
% <filtervisible> (optional) is whether to exclude
%   surfaces whose visibility setting is 'off' (but
%   which could still be visible in the render window
%   if you haven't redrawn yet).  if [] or not supplied,
%   default to 0.
% <filterpvalue> (optional) is whether to filter the
%   exported surfaces based on the p-value thresholding
%   settings.  if [] or not supplied, default to 0.
% <exportindices> (optional) is whether to export
%   indices into the associated data/pdata matrices
%   instead of the actual data/pdata values.
%
% the point of this function is to export the data and
% pdata corresponding to the regular surfaces on 
% a per-vertex basis.  you can use the <filtervisible>
% option to restrict which regular surfaces to export,
% the <filterpvalue> option to restrict the extent
% of the exported regular surfaces, and the <exportindices>
% option to get indices instead of values.
%
% this function returns <datacollect> and <pdatacollect>,
% both of which are of dimensions N x VNUM where N is the
% number of exported surfaces and VNUM is the number of vertices
% in the reference surface.  note that if <filtervisible> is 0,
% then N will be the total number of regular surfaces.
%
% for any surface that does not have corresponding pdata,
% the corresponding row in <pdatacollect> will consist of NaNs.
%
% for any vertex that is not mapped onto by data (or
% which doesn't satisfy the p-value threshold), the
% corresponding element in <datacollect> and the
% corresponding element in <pdatacollect> will be NaN.
%
% note that the rows of <datacollect> and <pdatacollect>
% are in a format that can be passed to viewsurfacedata, 
% namely the vertex-specification format.

% internal notes:
% - note that the combine & export functionality is dependent
% only on the reference surface and not on the target surface.

% FIXME: this function should be removed.  the user should deal with the data themselves.  see projecttovertices.
fprintf(1,'warning: it works but this function should not be used!!!!!');

global VS_GUI VS_PDATA VS_RVNUM;

% deal with input
if ~exist('filtervisible','var')
  filtervisible = 0;
end
if ~exist('filterpvalue','var')
  filterpvalue = 0;
end
if ~exist('exportindices','var')
  exportindices = 0;
end

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% determine number to combine
if filtervisible
  numcombine = 0;
  for p=handles.rsurf
    if handles.surfacerecord{p}.visibility
      numcombine = numcombine + 1;
    end
  end
else
  numcombine = length(handles.rsurf);
end

% init
datacollect = NaN*zeros([numcombine VS_RVNUM]);
pdatacollect = NaN*zeros([numcombine VS_RVNUM]);

% do it
cnt = 0;
for p=handles.rsurf
  if ~filtervisible || filtervisible && handles.surfacerecord{p}.visibility
    cnt = cnt + 1;
    if filterpvalue
      assert(0,'broken');
      % what were the indices lying completely within good p-thresholded voxels?
      ud = get(handles.surfs(p),'UserData');  % FIXME: BROEKN DUE TO FLATTENED!!!
      good = ud.vindices;
      % what are the corresponding data indices?
      [dindices,dummy] = viewsurfacedata_lookup(p-1,good);
      % sanity check
      assert(isequal(dummy,1:length(good)));
    else
      % what are all data indices?
      [dindices,good] = viewsurfacedata_lookup(p-1,-1);
    end
    % record
    if exportindices
      datacollect(cnt,good) = dindices;
      if ~isempty(VS_PDATA{p-1})
        pdatacollect(cnt,good) = dindices;
      end
    else
      datacollect(cnt,good) = viewsurfacedata_backtransform(p-1,dindices);
      if ~isempty(VS_PDATA{p-1})
        pdatacollect(cnt,good) = VS_PDATA{p-1}(dindices);
      end
    end
  end
end
