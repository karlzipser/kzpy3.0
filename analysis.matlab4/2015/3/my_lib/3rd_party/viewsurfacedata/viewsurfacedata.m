function viewsurfacedata(refsrf,targetsrf,data,datarange,tr, ...
                         xyzbegin,pdata,cmapnum,cmaptype)

% function viewsurfacedata(refsrf,targetsrf,data,datarange,tr, ...
%                          xyzbegin,pdata,cmapnum,cmaptype)
%
% <refsrf> is:
%   (1) the filename of the surface in respect to which <data>
%       is oriented.  no additional arguments are passed to
%       the surface loading function.
%   (2) a cell vector consisting of the arguments to the surface
%       loading function.  the first element must be like (1).
% 
% <targetsrf> is:
%   (1) the filename of the surface on which you want to
%       view <data> on.  no additional arguments are passed to
%       the surface loading function.
%   (2) a cell vector consisting of the arguments to the surface
%       loading function.  the first element must be like (1).
% 
% <data> is:
%   (1) this is the voxel-specification case.  <data> is 3D matrix of 
%       numbers (no NaNs or Infs!) in the same space that the functional 
%       slices were taken.  it doesn't have to have exactly the 
%       same dimensions as the functional slices --- see <xyzbegin>.
%   (2) 0, 1, 2, or 3.  these cause some nice test cases to be generated.
%       if 0, 1, or 2, you must specify a valid <tr> value.
%       if 0, 1, or 3, you may specify values for <cmapnum> and
%       <cmaptype> if you want.
%   (3) this is the vertex-specification case.  (horizontal) vector of
%       numbers which correspond to the vertices of <targetsrf>.  NaNs are
%       allowed and indicate to not color the associated vertices.
%   (4) cell vector of things like (1) or (3)
% 
% <datarange> (optional) is:
%   (1) a vector [lower upper], which allows you to explicitly set the 
%       lower and upper bounds of <data> for the purpose of colormap 
%       lookup.  note that values of <data> that lie outside of the
%       specified bounds are truncated to the appropriate bound!  
%       if [], we use the actual bounds of <data>.  
%       see the viewsurfacedata.txt document for more details.
%   (2) cell vector of things like (1).
% also, note that in the case that <data> is like (2), <datarange>
% is overridden.  if [] or not supplied, we assume [].
% 
% <tr> (optional) is:
%   (1) a result from maketransformation.m
%   (2) [], which means not supplied.
%   (3) cell vector of things like (1) or (2)
% in the case that <data> or its cell elements are of the
% vertex-specification case, then the associated <tr> is irrelevant
% and should be [].  in all other cases, a valid <tr> is required.  
% if [] or not supplied, we assume [].
% 
% <xyzbegin> (optional) is the matrix location (like [1 1 1] or 
% [12 20 1]) that <data> starts at within the epi volume.  if [] or
% not supplied, we assume [1 1 1].  can be a cell vector.  in the 
% case that <data> is like (2), <xyzbegin> is overridden.  in the 
% case that <data> or its cell elements are like (3), then <xyzbegin>
% is irrelevant and therefore ignored.
% 
% <pdata> (optional) is:
%   (1) matrix of numbers (no NaNs or Infs, but can be in any numerical 
%       range) corresponding to <data>.  this mirrors case (1)
%       of <data>.
%   (2) vector of numbers corresponding to the vertices of targetsrf.
%       this mirrors case (3) of <data>.  NaNs should exist exactly 
%       in the same spots as they occur in the corresponding <data>.
%   (3) [], which indicates that <pdata> is not specified
%   (4) cell vector of things like (1), (2), or (3)
% note that in the case that <data> is like (2), <pdata> is overridden.
%
% <cmapnum> (optional) is:
%   (1) a positive integer indicating the number of colors to use 
%       in the colormap
%   (2) cell vector of things like (1)
% if [] or not supplied, we default to 64.
%
% <cmaptype> (optional) is:
%   (1) 0, which means normal colormap mapping, in which the first 
%          and last values correspond to the beginning of the range 
%          associated with the first color and the end of the range
%          associated with the last color.  you probably want this for
%          normal, linear data.
%   (2) 1, which means colormap mapping where the first color is repeated
%          after the last color, and where the first and last
%          values correspond to the middle of the ranges associated with
%          the first color and (new) last color.  you may want this
%          for circular data when using an appropriately designed
%          colormap.
%   (3) cell vector of things like (1) or (2)
% if [] or not supplied, we default to 0.
%
% essentially, viewsurfacedata paints <data> on <targetsrf> according 
% to the spatial relationships between <data> and <refsrf> and using 
% <pdata> (if supplied) to determine thresholding.
%
% cell vectors are used to specify multiple data sets.  singleton inputs
% are automatically repeated as necessary to match the length of <data>.
% for example, if <data> is a three-element cell vector and <tr> is the
% output from maketransformation.m, then <tr> is automatically repeated so
% that the same transformation is applied to all three <data> matrices.

%%%%%%%%%%%%%%%%%%%%% preliminary stuff

% OBSOLETE
% % handle surfpak files
% if ~exist('~/.surfpak','dir')
%   assert(mkdir('~','.surfpak'),'failed to create ~/.surfpak directory!');
% end

% make sure clean slate (but preserve global vars!)
viewsurfacedata_closegui;
viewsurfacedata_disablerender;

%%%%%%%%%%%%%%%%%%%%% setup

% globals
global VS_REFSRF VS_RXYZ VS_RVNUM VS_RCOLORS VS_RNEIGHBORS VS_RISOLATED VS_RFACES;
global VS_TARGETSRF VS_TXYZ VS_TVNUM VS_TCOLORS VS_TNEIGHBORS VS_TISOLATED VS_TFACES;
global VS_TNORMALS;
global VS_SRFTYPE;
global VS_R VS_RPTR;
global VS_XYZBEGIN VS_XYZEND;
global VS_DATA VS_PDATA;
global VS_SPECIALBV VS_SPECIALSV VS_SPECIALCVLOW VS_SPECIALCVHIGH;
global VS_INTERVAL;
global VS_DRANGE VS_CDRANGE;
global VS_FIG VS_GUI;
global IRT_FACES;
global VS_CMAPTYPE;
global VS_CMAPNUM;
global VS_BEGINVALUE VS_ENDVALUE;
global VS_SELECTION;
global VS_TR;
global VS_OUTLINEC VS_OUTLINEE VS_OUTLINEV;

% internal constants
[figpos,guipos] = viewsurfacedata_constants('figpos','guipos');

%%%%%%%%%%%%%%%%%%%%% deal with input

% handle this special case
if ~iscell(data) && numel(data)==1
  % define data
  switch data
  case {0 1 2}
    % some defs
    if iscell(tr)
      [xdim,ydim,zdim] = separate(tr{1}.matrixsize);
    else
      [xdim,ydim,zdim] = separate(tr.matrixsize);
    end
    % ok do it
    switch data
    case 0
      data = { rand(xdim,ydim,zdim) };
      pdata = { rand(xdim,ydim,zdim) };
      datarange = [0 1];
    case 1
      data = { zeros(xdim,ydim,zdim) zeros(xdim,ydim,zdim) zeros(xdim,ydim,zdim) };
      data{1}(2:2:end,:,:) = 1;  % stripes==1 along x-direction
      data{2}(:,2:2:end,:) = 1;  % stripes==1 along y-direction
      data{3}(:,:,2:2:end) = 1;  % stripes==1 along z-direction
      pdata = data;
      datarange = [0 1];
    case 2
      data = { zeros(xdim,ydim,zdim) zeros(xdim,ydim,zdim) zeros(xdim,ydim,zdim) };
      data{1}(2:2:end,:,:) = 1;  % stripes==1 along x-direction
      data{2}(:,2:2:end,:) = 2;  % stripes==2 along y-direction
      data{3}(:,:,2:2:end) = 4;  % stripes==4 along z-direction
      data = data{1}+data{2}+data{3};
      pdata = data;
      datarange = [-.5 7.5];
      cmapnum = 8;
      cmaptype = 0;
    end
    xyzbegin = [];
  case 3
    % we don't know how many vertices there are yet, so we can't
    % set data and pdata correctly.  deal with this special case later.  UGLY.
    data = [];
    pdata = [];
    datarange = [0 1];
  otherwise
    assert(0,'bad <data> input.\n');
  end
end

% handle optional inputs
if ~exist('datarange','var')
  datarange = [];
end
if ~exist('tr','var')
  tr = [];
end
if ~exist('xyzbegin','var')
  xyzbegin = [];
end
if ~exist('pdata','var')
  pdata = [];
end
if ~exist('cmapnum','var')
  cmapnum = [];
end
if ~exist('cmaptype','var')
  cmaptype = [];
end

%%%%%%%%%%%%%%%%%%%%%% manipulations...

% get into our preferred format
if ~iscell(refsrf)
  refsrf = {refsrf};
end
if ~iscell(targetsrf)
  targetsrf = {targetsrf};
end
if iscell(data)
  VS_DATA = data;
else
  VS_DATA = {data};
end
clear data;  % save memory
if iscell(datarange)
  VS_DRANGE = datarange;
else
  VS_DRANGE = {datarange};
end
if iscell(tr)
  VS_TR = tr;
else
  VS_TR = {tr};
end
if iscell(xyzbegin)
  VS_XYZBEGIN = xyzbegin;
else
  VS_XYZBEGIN = {xyzbegin};
end
if iscell(pdata)
  VS_PDATA = pdata;
else
  VS_PDATA = {pdata};
end
clear pdata;  % save memory
if iscell(cmapnum)
  VS_CMAPNUM = cmapnum;
else
  VS_CMAPNUM = {cmapnum};
end
if iscell(cmaptype)
  VS_CMAPTYPE = cmaptype;
else
  VS_CMAPTYPE = {cmaptype};
end

% NOTE: at this point, we are using VS_* vars

% allow for single length things
if length(VS_DRANGE)==1
  VS_DRANGE = repmat(VS_DRANGE,1,length(VS_DATA));
end
if length(VS_TR)==1
  VS_TR = repmat(VS_TR,1,length(VS_DATA));
end
if length(VS_XYZBEGIN)==1
  VS_XYZBEGIN = repmat(VS_XYZBEGIN,1,length(VS_DATA));
end
if length(VS_PDATA)==1
  VS_PDATA = repmat(VS_PDATA,1,length(VS_DATA));
end
if length(VS_CMAPNUM)==1
  VS_CMAPNUM = repmat(VS_CMAPNUM,1,length(VS_DATA));
end
if length(VS_CMAPTYPE)==1
  VS_CMAPTYPE = repmat(VS_CMAPTYPE,1,length(VS_DATA));
end

% check lengths
assert(length(VS_DRANGE)==length(VS_DATA),'number of cell elements of <datarange> is incorrect');
assert(length(VS_TR)==length(VS_DATA),'number of cell elements of <tr> is incorrect');
assert(length(VS_XYZBEGIN)==length(VS_DATA),'number of cell elements of <xyzbegin> is incorrect');
assert(length(VS_PDATA)==length(VS_DATA),'number of cell elements of <pdata> is incorrect');
assert(length(VS_CMAPNUM)==length(VS_DATA),'number of cell elements of <cmapnum> is incorrect');
assert(length(VS_CMAPTYPE)==length(VS_DATA),'number of cell elements of <cmaptype> is incorrect');

% handle defaults
for p=1:length(VS_DATA)
  if isempty(VS_DRANGE{p})
    minv = min(VS_DATA{p}(:));
    maxv = max(VS_DATA{p}(:));
    VS_DRANGE{p} = [minv maxv];
    fprintf(1,'for data number %d, using minimum value %f and maximum value %f for <datarange>.\n',p,minv,maxv);
  end
  if VS_DRANGE{p}(1) >= VS_DRANGE{p}(2)
    fprintf(1,'error: the <datarange> for data number %d must be like [a b] where a < b.\n',p);
    return;
  end
end
for p=1:length(VS_XYZBEGIN)
  if isempty(VS_XYZBEGIN{p})
    VS_XYZBEGIN{p} = [1 1 1];
  end
end
for p=1:length(VS_DATA)
  if ~isempty(VS_TR{p})
    assert(ndims(VS_DATA{p})==3,['data number ',num2str(p),' has unexpected dimensions (expected a 3D matrix)']);
    VS_XYZEND{p} = VS_XYZBEGIN{p}+size(VS_DATA{p})-1;
    if isfield(VS_TR{p},'extra')
      ms = VS_TR{p}.extra.matrixsize;
    else
      ms = VS_TR{p}.matrixsize;
    end
    assert(all(VS_XYZEND{p}<=ms),['data number ',num2str(p),' has unexpected dimensions (check <xyzbegin>?)']);
  end
end
for p=1:length(VS_CMAPNUM)
  if isempty(VS_CMAPNUM{p})
    VS_CMAPNUM{p} = viewsurfacedata_constants('cmapnum');
  end
end
for p=1:length(VS_CMAPTYPE)
  if isempty(VS_CMAPTYPE{p})
    VS_CMAPTYPE{p} = 0;
  end
end

% pre-process inputs
refsrf{1} = absolutepath(refsrf{1});
targetsrf{1} = absolutepath(targetsrf{1});

%%%%%%%%%%%%%%%%%%%%% load .srf files

% if necessary, load reference surface
if isequal(VS_REFSRF,refsrf{1})
  fprintf(1,'using previously read <refsrf> values.\n');
  assert(~isempty(VS_SRFTYPE));
  assert(~isempty(VS_RXYZ));
  assert(~isempty(VS_RFACES));
  assert(~isempty(VS_RCOLORS));
  assert(~isempty(VS_RNEIGHBORS));
  assert(~isempty(VS_RISOLATED));
else
  [dummy,dummy,ext] = fileparts(refsrf{1});
  fprintf(1,'loading <refsrf>.\n');
  switch ext
  case '.srf'
    VS_SRFTYPE = 1;
    [VS_RXYZ,VS_RFACES,VS_RCOLORS] = loadsrf(refsrf{:});
  case '.vtk'
    VS_SRFTYPE = 2;
    [VS_RXYZ,VS_RFACES,VS_RCOLORS] = loadvtk(refsrf{:});
  case '.mat'
    VS_SRFTYPE = 3;
    temp = load(refsrf{1},'vertices','faces');
    VS_RXYZ = temp.vertices;
    VS_RFACES = temp.faces;
    VS_RCOLORS = zeros(1,size(VS_RXYZ,2));
    clear temp;
  otherwise
    error('unknown <refsrf> file type');
  end
  [VS_RNEIGHBORS,VS_RISOLATED] = facestoneighbors(VS_RFACES,size(VS_RXYZ,2));
  VS_REFSRF = refsrf{1};
  % this is a crucial step.  given that the reference surface
  % changed, then all bets are off for using previous
  % calculations stored in VS_R.
  VS_R = {};
end
VS_RVNUM = size(VS_RXYZ,2);

% ok, now that we have the reference surface loaded in, we can deal with the special <data>==3 case.
if isempty(VS_DATA{1})
  VS_DATA{1} = rand(1,VS_RVNUM);
  VS_PDATA{1} = rand(1,VS_RVNUM);
end

% ok, now deal with VS_R (which may already exist from a previous call).
% VS_RPTR{x} is the index into VS_R for data number x.  if [], that indicates the vertex-specification case.
VS_RPTR = {};
for p=1:length(VS_TR)
  % if vector case
  if isempty(VS_TR{p})
    VS_RPTR{p} = [];
    % sanity check
    assert(size(VS_DATA{p},2)==numel(VS_DATA{p}) && length(VS_DATA{p})==VS_RVNUM, ...
           ['data number ',num2str(p),' has unexpected dimensions (remember that we expect a (horizontal) vector)']);
    % sanity check
    if ~isempty(VS_PDATA{p})
      assert(size(VS_PDATA{p},2)==numel(VS_PDATA{p}) && length(VS_PDATA{p})==VS_RVNUM, ...
             ['pdata number ',num2str(p),' has unexpected dimensions (remember that we expect a (horizontal) vector)']);
    end
  else
    % see if we've already done this one (hinging upon tr, xyzbegin, and xyzend)
    found = 0;
    for q=1:length(VS_R)
      if isequal(VS_TR{p},VS_R{q}.tr) && ...
         isequal(VS_XYZBEGIN{p},VS_R{q}.xyzbegin) && ...
         isequal(VS_XYZEND{p},VS_R{q}.xyzend)
        found = 1;
        VS_RPTR{p} = q;
        fprintf(1,'using cached transformations for data number %d.\n',p);
        break;
      end
    end
    if found
      continue;
    end
    % ok, we haven't done it
    idx = length(VS_R)+1;
    % point to the solution we are about to make
    VS_RPTR{p} = idx;
    % fill out the effective "index" fields
    VS_R{idx}.tr = VS_TR{p};
    VS_R{idx}.xyzbegin = VS_XYZBEGIN{p};
    VS_R{idx}.xyzend = VS_XYZEND{p};
    % put in a tr-related field
    if isfield(VS_R{idx}.tr,'extra')  % DEPRECATED
      VS_R{idx}.matrixsizereal = VS_R{idx}.tr.extra.matrixsize;
    else
      VS_R{idx}.matrixsizereal = VS_R{idx}.tr.matrixsize;
    end
    % additional calc
    VS_R{idx}.xyzsize = VS_XYZEND{p}-(VS_XYZBEGIN{p}-1);
    % sanity checks
    assert(isequal(size(VS_DATA{p}),VS_R{idx}.xyzsize),['data number ',num2str(p),' has unexpected dimensions']);
    if ~isempty(VS_PDATA{p})
      assert(isequal(size(VS_PDATA{p}),VS_R{idx}.xyzsize),['pdata number ',num2str(p),' has unexpected dimensions']);
    end
    % project vertices to gems space, then project to epi space
    if isfield(VS_R{idx}.tr,'sbv_reorder')  % TODO: get rid of this legacy!!!!
      VS_R{idx}.raw = gemstoepi(bvtogems(VS_RXYZ,VS_R{idx}.tr),VS_R{idx}.tr);
    else
      VS_R{idx}.raw = volumetoslices(VS_RXYZ,VS_R{idx}.tr);
    end
    % round to find nearest epi pixel
    VS_R{idx}.rnd = round(VS_R{idx}.raw(1:3,:));
    % pull out data and vertex indices of all functional intersections (note that nan filtering is not done)
    [VS_R{idx}.indices,VS_R{idx}.valid] = viewsurfacedata_lookup(p,-1);
    % check sanity
    if isempty(VS_R{idx}.valid)
      fprintf(1,'warning: no surface vertices mapped to data volume number %d.\n',p);
    end
    % note that stuff related to voxel boundaries, voxel connections, and partial-volume voxels
    % get stored in VS_R at a later time.  these are calculated when needed and then reused.
  end
end

% at this point, we prefer to use VS_R rather than VS_TR,VS_XYZBEGIN,VS_XYZEND

% if necessary, load target surface
if isequal(VS_TARGETSRF,targetsrf{1})
  fprintf(1,'using previously read <targetsrf> values.\n');
  assert(~isempty(VS_TXYZ));
  assert(~isempty(VS_TFACES));
  assert(~isempty(VS_TCOLORS));
  assert(~isempty(VS_TNEIGHBORS));
  assert(~isempty(VS_TISOLATED));
else
  % well, if it's the same as the ref, just use the already read-in values
  if isequal(targetsrf{1},VS_REFSRF)
    fprintf(1,'since <targetsrf> is <refsrf>, using <refsrf> values for <targetsrf> values.\n');
    VS_TXYZ = VS_RXYZ;
    VS_TFACES = VS_RFACES;
    VS_TCOLORS = VS_RCOLORS;
    VS_TNEIGHBORS = VS_RNEIGHBORS;
    VS_TISOLATED = VS_RISOLATED;
  else
    [dummy,dummy,ext] = fileparts(targetsrf{1});
    fprintf(1,'loading <targetsrf>.\n');
    switch ext
    case '.srf'
      [VS_TXYZ,VS_TFACES,VS_TCOLORS] = loadsrf(targetsrf{:});
    case '.vtk'
      [VS_TXYZ,VS_TFACES,VS_TCOLORS] = loadvtk(targetsrf{:});
    case '.mat'
      temp = load(targetsrf{1},'vertices','faces');
      VS_TXYZ = temp.vertices;
      VS_TFACES = temp.faces;
      VS_TCOLORS = zeros(1,size(VS_TXYZ,2));
      clear temp;
    otherwise
      error('unknown <targetsrf> file type');
    end
    [VS_TNEIGHBORS,VS_TISOLATED] = facestoneighbors(VS_TFACES,size(VS_TXYZ,2));
  end
  VS_TARGETSRF = targetsrf{1};
end
VS_TVNUM = size(VS_TXYZ,2);

% OBSOLETE BV STUFF
% report some info
%if VS_SRFTYPE==1 && ismember(-1000,VS_TCOLORS)
%  fprintf(1,'target surface seems to be (BV-)flattened.\n');
%  temp = VS_TCOLORS>=0;
%  VS_TCOLORS(temp) = mod(VS_TCOLORS(temp),2);  % transform any values >= 10000 down to 0/1 (TODO: revisit)
%else
%  fprintf(1,'target surface seems to be NOT flattened.\n');
%end

% report
fprintf(1,'\nsome information:\n');
fprintf(1,'reference: %d vertices, with %d isolated vertices.\n',VS_RVNUM,count(VS_RISOLATED));
fprintf(1,'   target: %d vertices, with %d isolated vertices.\n',VS_TVNUM,count(VS_TISOLATED));
fprintf(1,'reference: %d faces.\n',size(VS_RFACES,1));
fprintf(1,'   target: %d faces.\n',size(VS_TFACES,1));
if any(VS_TCOLORS>0)
  fprintf(1,'curvature information detected in target surface.\n');  % TODO: revisit?
else
  fprintf(1,'curvature information not detected in target surface.\n');
end
fprintf(1,'\n');

% check ref<->target number of vertices
assert(VS_RVNUM==VS_TVNUM,'ref .srf and target .srf must have the same number of vertices!');

% no need to check neighbors since dependent on faces
% no need to check colors since target need not have the same color information as reference

% OBSOLETE BV STUFF
%% ok, weird BV flattened brain handling (it is important that this happens exactly here)
%if VS_SRFTYPE==1 && ismember(-1000,VS_TCOLORS)  % if BV-flattened (TODO: revisit)
%  % if any face involves the weird -1000 or -10000 color value, remove it PERMANENTLY
%  temp = VS_TCOLORS(VS_TFACES);
%  bad = sum((temp==-1000 | temp==-10000),2) ~= 0;
%  VS_TFACES(bad,:) = [];
%end

% deal with intersectraytri setup (for speed, this could be by-passed if we knew that the first surface was the same as last time)
% setup the global var for intersectraytri
IRT_FACES = permute(reshape(VS_TXYZ(1:3,VS_TFACES),[3 size(VS_TFACES,1) 3]),[2 3 1]);
% call it once initially to allow the new value for IRT_FACES to be registered with intersectraytri.
intersectraytri([0 0 0],[0 0 1],1);

%%%%%%%%%%%%%%%%%%%%% calculations

% note that we do this with respect to the first data.
switch VS_CMAPTYPE{1}
case 0
  VS_INTERVAL = (VS_DRANGE{1}(2) - VS_DRANGE{1}(1))/VS_CMAPNUM{1};
case 1
  VS_INTERVAL = (VS_DRANGE{1}(2) - VS_DRANGE{1}(1))/(VS_CMAPNUM{1}+1);
end

% for each data, there is a corrected data range that the data must be fit into
VS_CDRANGE = {};
for p=1:length(VS_DRANGE)
  if p==1
    switch VS_CMAPTYPE{1}
    case 0
      VS_CDRANGE{1} = VS_DRANGE{1};
    case 1
      VS_CDRANGE{1}(1) = VS_DRANGE{1}(1) + VS_INTERVAL/2;
      VS_CDRANGE{1}(2) = VS_DRANGE{1}(2) - VS_INTERVAL/2;
    end
    endvalue = VS_DRANGE{1}(2) + VS_INTERVAL;  % this tracks the last encountered ending value after the upper padding
  else
    switch VS_CMAPTYPE{p}
    case 0
      x1 = endvalue + VS_INTERVAL;
      endvalue = endvalue + (VS_CMAPNUM{p}+2)*VS_INTERVAL;  % one lower padding, one upper padding
    case 1
      x1 = endvalue + VS_INTERVAL + VS_INTERVAL/2;
      endvalue = endvalue + (VS_CMAPNUM{p}+3)*VS_INTERVAL;  % one lower padding, one upper padding, one extra b/c of the 1/2 split thing
    end
    x2 = x1+VS_CMAPNUM{p}*VS_INTERVAL;
    VS_CDRANGE{p} = [x1 x2];
  end
end

% transform data to new colormap ranges
for p=1:length(VS_DATA)
  VS_DATA{p} = normalizerange(VS_DATA{p},VS_CDRANGE{p}(1),VS_CDRANGE{p}(2),VS_DRANGE{p}(1),VS_DRANGE{p}(2),1);  % note that we chop!
end

% determine the special selection color
VS_SPECIALSV = {};
VS_SPECIALSV{1} = VS_DRANGE{1}(1)-VS_INTERVAL/2 - 5*VS_INTERVAL;  % note that this is 1-indexed not sval-indexed

% determine the special brain colors
VS_SPECIALBV = VS_DRANGE{1}(1)-VS_INTERVAL/2 - 4*VS_INTERVAL;

% determine the special curvature colors
VS_SPECIALCVLOW = VS_DRANGE{1}(1)-VS_INTERVAL/2 - 3*VS_INTERVAL;
VS_SPECIALCVHIGH = VS_DRANGE{1}(1)-VS_INTERVAL/2 - 2*VS_INTERVAL;

% determine the special partial volume color
dummy = VS_DRANGE{1}(1)-VS_INTERVAL/2 - VS_INTERVAL;  % well, just a placeholder

% define
VS_BEGINVALUE = VS_SPECIALSV{end}-VS_INTERVAL/2;
VS_ENDVALUE = endvalue;

%%%%%%%%%%%%%%%%%%%%% main stuff

% handle figure stuff
  % make sure invisible to begin with because the creation of the gui window causes things to be rendered prematurely i think
VS_FIG = figure('Visible','off');
setfigurepos(VS_FIG,figpos);
set(VS_FIG,'DoubleBuffer','on');  % doublebuffer has no effect in opengl mode?
set(VS_FIG,'CloseRequestFcn','delete(gcf); viewsurfacedata_closegui;');
set(VS_FIG,'Pointer','custom');
set(VS_FIG,'PointerShapeCData',pointercrosssmall);
set(VS_FIG,'PointerShapeHotSpot',[5 5]);
hold on;

% aux stuff (this is important to do early)
assert(VS_BEGINVALUE ~= VS_ENDVALUE);
set(gca,'CLim',[VS_BEGINVALUE VS_ENDVALUE],'CLimMode','manual');
set(gca,'ALim',[0 1],'ALimMode','manual');  % not really necessary since we use alphadatamapping=='none'

% handle brain surface
surfs = [viewsurfacedata_drawsurface(1,[])];

% immediately deal with TNORMALS.  it is important this be done ASAP since other things depend on this information.
temp = unitlength(get(surfs(1),'VertexNormals'),2);  % N x 3
set(surfs(1),'VertexNormals',temp);
VS_TNORMALS = [temp'; ones(1,size(temp,1))];  % now 4 x N (so as to parallel VS_*XYZ)

% handle regular surfaces
for p=1:length(VS_DATA)
  % note that below is [].  we can afford to do this because we call drawsurface again later with correct values.
  % also note that boundary is 2.  i think we can afford to do this for the same reasons as above...?
  surfs = [surfs viewsurfacedata_drawsurface(2,[],p,1,2,[],1,2,2)];
end

% handle selection case
surfs = [surfs viewsurfacedata_drawsurface(3,[],1,[],[],1,2,2)];

% handle extra special curvature surface case
if ~all(VS_TCOLORS==0)
  surfs = [surfs viewsurfacedata_drawsurface(4,[],1,2,[],1,4,2)];
end

% create placeholder for partial volume (which never gets used)
if ~isempty(VS_RPTR{1})
  surfs = [surfs patchempty];
end

% create placeholders for voxel boundaries (which do get used)
if ~isempty(VS_RPTR{1})
  surfs = [surfs patchempty patchempty patchempty];
end

% we would create partial-volume objects here... were it not for the fact
% that we have to create new line objects on each camera change.  so
% we don't do anything here.

% draw selector points
%%pt1 = plot3([NaN;NaN],[NaN;NaN],[NaN;NaN],'c-');
%%pt1tip = scatter3(NaN,NaN,NaN,'cx');
%%pt2 = plot3([NaN;NaN],[NaN;NaN],[NaN;NaN],'g-');
%%pt2tip = scatter3(NaN,NaN,NaN,'gx');
ptclick = scatter3(NaN,NaN,NaN,'c.');

% FIXME
%% draw axesdir for target srf
%axesdir = viewsurfacedata_drawaxesdir([]);

% aux stuff
axis vis3d;
set(gca,'Visible','off');
%set(gca,'Visible','on');
%set(gca,'Color','none')   
%xlabel('x');
%ylabel('y');
%zlabel('z');
%set(gca,'XColor','red');
%set(gca,'YColor','green');
%set(gca,'ZColor','blue');
%set(gca,'XGrid','on');
%set(gca,'YGrid','on');
%set(gca,'ZGrid','on');
set(gca,'DataAspectRatio',[1 1 1],'DataAspectRatioMode','manual');

% camera stuff
view(0,0);
camproj('perspective');
set(gca,'CameraPositionMode','manual');
set(gca,'CameraTargetMode','manual');
set(gca,'CameraUpVectorMode','manual');
set(gca,'CameraViewAngleMode','manual');
set(gca,'PlotBoxAspectRatioMode','manual');
set(gca,'XLimMode','manual');
set(gca,'YLimMode','manual');
set(gca,'ZLimMode','manual');

% ok, this is crucial.  sometimes the plotboxaspectratio is highly skewed
% because the surface itself was highly anamorphic.  by doing the following
% step we ensure that the plotboxaspectratio becomes [1 1 1].  essentially,
% we scale the x- y- and z-limits to match the axis with greatest range.
% the fact that this happens here is important; MATLAB must have did some
% pre-calculations before this point to get them how they are.  i thought
% that this change could have messed camera presets, but i guess not!!!
ar = get(gca,'PlotBoxAspectRatio');
set(gca,'XLim',scalerange(get(gca,'XLim'),max(ar)/ar(1)));
set(gca,'YLim',scalerange(get(gca,'YLim'),max(ar)/ar(2)));
set(gca,'ZLim',scalerange(get(gca,'ZLim'),max(ar)/ar(3)));

% light stuff (we do a findobj later on)
camlight('headlight','infinite');

% no more hold
hold off;

% gui stuff
VS_GUI = viewsurfacedata_gui(VS_FIG,surfs,ptclick);
setfigurepos(VS_GUI,guipos);
set(VS_GUI,'CloseRequestFcn','delete(gcf); viewsurfacedata_disablerender;');

% force a redraw
handles = guidata(VS_GUI);
viewsurfacedata_gui('redraw_Callback',handles.redraw,[],handles);

% ok, make visible
set(VS_FIG,'Visible','on');

% deal with opengl
if isequal(get(VS_FIG,'Renderer'),'OpenGL')
  fprintf(1,'rendering mode is OpenGL (yay).\n');
else
  fprintf(1,['warning: rendering mode is not OpenGL but is ',get(VS_FIG,'Renderer'),'.\n']);
end

% report finished
fprintf(1,'viewsurfacedata executed successfully.\n');
