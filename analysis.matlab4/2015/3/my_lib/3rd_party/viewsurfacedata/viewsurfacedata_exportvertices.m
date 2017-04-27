function [xx,yy,zz,dd] = viewsurfacedata_exportvertices

% function [xx,yy,zz,dd] = viewsurfacedata_exportvertices
%
% this function returns <xx>, <yy>, and <zz>, each of which
% are of dimensions N x VNUM where N is the number of data sets
% (i.e. regular surfaces) and VNUM is the number of vertices 
% in the reference surface.  these matrices indicate the
% coordinates of vertices in respect to each of the data sets.
% note that these coordinates do not reflect the <xyzbegin> 
% feature (i.e., the coordinates are relative to the original,
% real voxel space).
%
% this function also returns <dd>, which is of dimensions
% N x VNUM.  this matrix indicates the corresponding index 
% into the data volume for each vertex.  for example, 
% dd(3,10340) is the data index for vertex 10340 in the
% case of regular surface number 3.  if a vertex doesn't
% lie within a data voxel, then its value is NaN.
%
% for a data set that was specified via the vertex method, all
% entries for the corresponding rows in <xx>, <yy>, <zz>, and <dd>
% will be NaN.

global VS_GUI VS_RVNUM VS_RPTR VS_R;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% define
handles = guidata(VS_GUI);

% init
xx = zeros([length(handles.rsurf) VS_RVNUM]);
yy = zeros([length(handles.rsurf) VS_RVNUM]);
zz = zeros([length(handles.rsurf) VS_RVNUM]);
dd = zeros([length(handles.rsurf) VS_RVNUM]);

% do it
for p=1:length(handles.rsurf)
  % if vertex case     %%% FIXME: is this a correct way to determine vertex case?
  if isempty(VS_RPTR{p})
    xx(p,:) = repmat(NaN,1,VS_RVNUM);
    yy(p,:) = repmat(NaN,1,VS_RVNUM);
    zz(p,:) = repmat(NaN,1,VS_RVNUM);
    dd(p,:) = repmat(NaN,1,VS_RVNUM);
  % if voxel case
  else
    xx(p,:) = VS_R{VS_RPTR{p}}.raw(1,:);
    yy(p,:) = VS_R{VS_RPTR{p}}.raw(2,:);
    zz(p,:) = VS_R{VS_RPTR{p}}.raw(3,:);

    % init
    dd(p,:) = repmat(NaN,1,VS_RVNUM);

    % NOTE: this mirrors vsd_lookup!
    rnd = VS_R{VS_RPTR{p}}.rnd;
    xyzbegin = VS_R{VS_RPTR{p}}.xyzbegin;
    xyzend = VS_R{VS_RPTR{p}}.xyzend;
    xyzsize = VS_R{VS_RPTR{p}}.xyzsize;
    good = find(rnd(1,:) >= xyzbegin(1) & rnd(1,:) <= xyzend(1) & ...
                rnd(2,:) >= xyzbegin(2) & rnd(2,:) <= xyzend(2) & ...
                rnd(3,:) >= xyzbegin(3) & rnd(3,:) <= xyzend(3));
    if isempty(good)
      dindices = [];
    else
      dindices = sub2ind(xyzsize,rnd(1,good)-xyzbegin(1)+1,...  % FIXME: can we use sub2ind2?
                                 rnd(2,good)-xyzbegin(2)+1,...
                                 rnd(3,good)-xyzbegin(3)+1);
    end

    % assign good vertices their corresponding data indices    
    dd(p,good) = dindices;
  end
end
