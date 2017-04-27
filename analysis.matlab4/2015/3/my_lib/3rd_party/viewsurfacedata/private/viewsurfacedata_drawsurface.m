function pobj = viewsurfacedata_drawsurface(casenum,pobj,x,y,z,a,b,c,d)

% function pobj = viewsurfacedata_drawsurface(casenum,pobj,x,y,z,a,b,c,d)
%
% in all cases, pobj is an existing patch handle to mangle.
% if [], then we make a new patch.
%
% case 1: viewsurfacedata_drawsurface(1,pobj)
%   this is the brain surface case
%
% case 2: viewsurfacedata_drawsurface(2,pobj,p,pthresh,pthreshdir,below,belowp,belowpdir,boundary)
%   p is the number of a regular surface
%   pthresh is a p-threshold.
%   pthreshdir is 1, 2, 3, or 4
%   below,belowp,belowpdir are as in viewsurfacedata_lookupbelow.m.
%   boundary==1 means expand, ==2 means restrict.  cannot be ==3!
%
% case 3: viewsurfacedata_drawsurface(3,pobj,p,vindices,below,belowp,belowpdir,boundary)
%   p is the 1-indexed number corresponding to this selection surface
%   vindices is a vector of vertex indices
%   below,belowp,belowpdir are as in viewsurfacedata_lookupbelow.m.
%   boundary==1 means expand, ==2 means restrict, ==3 means outline
%
% case 4: viewsurfacedata_drawsurface(4,pobj,pthresh,pthreshdir,below,belowp,belowpdir,boundary)
%   this is the curvature case.  should call me
%   only when the colors aren't all 0, although i won't crash 
%   if you do.  the inputs are like in the other cases.
%
% draw and return a handle to a patch object.
%
% note the subtle TISOLATED handling in here!  it's voodoo.

global VS_TFACES VS_TXYZ VS_TNORMALS VS_TCOLORS VS_RNEIGHBORS;
global VS_PDATA VS_DATA VS_R VS_RPTR;
global VS_SPECIALBV VS_SPECIALSV;
global VS_SPECIALCVLOW VS_SPECIALCVHIGH;
global VS_TISOLATED;

% handle input
if isempty(pobj)
  pobj = patch;
end

% define other stuff
if casenum==1
end
if casenum==2
  p = x;
  pthresh = y;
  pthreshdir = z;
  below = a;
  belowp = b;
  belowpdir = c;
  boundary = d;
end
if casenum==3
  p = x;
  vindices = y;
  below = z;
  belowp = a;
  belowpdir = b;
  boundary = c;
end
if casenum==4
  pthresh = x;
  pthreshdir = y;
  below = z;
  belowp = a;
  belowpdir = b;
  boundary = c;
end

if casenum==1
  faces = VS_TFACES;
  vertices = VS_TXYZ(1:3,:)';
  % here, note that there is no vertexnormals, since we need to calc it!
  vertexdata = repmat(VS_SPECIALBV,[1 size(vertices,1)]);
  fvad = ones(1,size(vertices,1));
end

if casenum==2
  % calculate vindices, extracting only those that satisfy p-threshold
  [dindices,vindices] = viewsurfacedata_access(p);
  % well, do we have p-values?
  if ~isempty(VS_PDATA{p})
    % amazingly, this works for both cases
    switch pthreshdir
    case 1
      vindices = vindices(VS_PDATA{p}(dindices) < pthresh);
    case 2
      vindices = vindices(VS_PDATA{p}(dindices) <= pthresh);
    case 3
      vindices = vindices(VS_PDATA{p}(dindices) > pthresh);
    case 4
      vindices = vindices(VS_PDATA{p}(dindices) >= pthresh);
    end
  end
  % remove isolated
  vindices = vindices(~VS_TISOLATED(vindices));
end

if casenum==3
  % remove isolated
  vindices = vindices(~VS_TISOLATED(vindices));
  % handle special case
  if boundary==3
    pobj = viewsurfacedata_drawoutline(pobj,vindices);  % a little ugly.  pass off to a different function.
    return;
  end
end

if casenum==4
  % calculate vindices, extracting only those that satisfy p-threshold
  switch pthreshdir
  case 1
    vindices = find(VS_TCOLORS < pthresh);
  case 2
    vindices = find(VS_TCOLORS <= pthresh);
  case 3
    vindices = find(VS_TCOLORS > pthresh);
  case 4
    vindices = find(VS_TCOLORS >= pthresh);
  end
  % remove isolated
  vindices = vindices(~VS_TISOLATED(vindices));
end

if casenum==2 || casenum==3 || casenum==4
  % which faces are relevant
  temp = sum(ismember(VS_TFACES,vindices),2);
  goodfaces = find(temp==3);  % all three

  % deal with boundary flag
  switch boundary
  case 1
    allfaces = find(temp>0);  % at least one vertex
    okfaces = setdiff(allfaces,goodfaces);  % between one and two
  case 2
    allfaces = goodfaces;
    okfaces = [];
  % note that case 3 is explicitly handled above
  end

  % which vertices are involved in a relevant
  % face but are NOT one of the ones that the user specifically
  % wanted painted?
  okindices = setdiff(flatten(VS_TFACES(okfaces,:)),vindices);
  
  % all vertices that we currently care about
  allindices = union(vindices,okindices);

  % vindices are opaque (alpha=1), okindices are invisible (alpha=0)
  fvad = numreplace2(allindices,[vindices okindices],[ones(1,length(vindices)) zeros(1,length(okindices))]);
  
  % construct faces, and include fake degenerate faces for anything in vindices that didn't get involved.
  % the latter scenario is expected only in the boundary==2 case...
  faces = [VS_TFACES(allfaces,:); repmat(setdiff(vindices(:),vflatten(VS_TFACES(allfaces,:))),[1 3])];

  % re-index
  faces = numreplace2(faces,allindices);
  
  % define more
  vertices = VS_TXYZ(1:3,allindices)';
  vertexnormals = VS_TNORMALS(1:3,allindices)';

  % first, figure out dvalues for vindices
  if casenum==2
    [dindices,good] = viewsurfacedata_lookup(p,vindices);
    assert(length(dindices)==length(vindices));  % sanity check that all were good
    vdvalues = VS_DATA{p}(dindices);

%% different vertex asst:
%dindices = [];
%good = [];
%vsr = VS_R{VS_RPTR{p}};
%[xx,yy,zz] = ndgrid(vsr.xyzbegin(1):vsr.xyzend(1),vsr.xyzbegin(2):vsr.xyzend(2),vsr.xyzbegin(3):vsr.xyzend(3));
%vdvalues = flatten(interpn(xx,yy,zz,VS_DATA{p},vsr.raw(1,vindices),vsr.raw(2,vindices),vsr.raw(3,vindices),'linear'));
%% what about edge effects (no data there)?
%keyboard;

  end
  if casenum==3
    vdvalues = repmat(VS_SPECIALSV{p},[1 length(vindices)]);
  end
  if casenum==4
    vdvalues = normalizerange(VS_TCOLORS(vindices),VS_SPECIALCVLOW,VS_SPECIALCVHIGH,0,1,1);  % the chop probably isn't necessary
  end
  
  % then, figure out dvalues for okindices
  okdvalues = viewsurfacedata_lookupbelow(okindices,below,belowp,belowpdir);

  % finally, put them all back together again!
  vertexdata = numreplace2(allindices,[vindices okindices],[vdvalues okdvalues]);
end

% handle weird degenerate case
if isempty(vertices)
  vertices = [];  % explicitly change from 0x3 to 0x0
end

% this casenum==1 handling is just so we can have MATLAB calculate normals.
% all other cases leech from the calculated normals from the casenum==1 case,
% so obviously, order of calls to _drawsurface matter!
if casenum==1
  set(pobj,    'Faces',faces,                    ...
               'Vertices',vertices,              ...  % this is without the normalshift adjustment (to come later)
               'NormalMode','auto');
else
  set(pobj,    'Faces',faces,                    ...
               'Vertices',vertices,              ...  % this is without the normalshift adjustment (to come later)
               'VertexNormals',vertexnormals,    ...
               'NormalMode','manual');
end
set(pobj,    'FaceVertexCData',vertexdata(:),  ...
             'FaceVertexAlphaData',fvad(:),    ...
             'CDataMapping','scaled',          ...  % values interpreted with respect to clim
             'AlphaDataMapping','none',        ...  % values are raw in [0,1]
             'BackFaceLighting','unlit',       ...  % back cases probably never really happen; let's do unlit for speed
             'EdgeLighting','none',            ...  % for speed
             'MarkerSize',  2,                   ...
             'MarkerEdgeColor','white',        ...
             'AmbientStrength',0.3,            ...  % these values are equivalent to "material dull"
             'DiffuseStrength',0.8,            ...
             'SpecularStrength',0,             ...
             'SpecularExponent',10,            ...
             'SpecularColorReflectance',1,     ...
             'LineWidth',1,                    ...
             'Clipping','on');

% additional setting
if casenum==1 || casenum==4  % curvature doesn't get vindices since we won't be averaging data with it
  set(pobj,'UserData',struct('origvertices',vertices));
end
if casenum==1
  set(pobj,'FaceColor','flat');  % for speed
end
if casenum==2 || casenum==3 || casenum==4
  set(pobj,'FaceColor','interp');
end
if casenum==2
  set(pobj,'UserData',struct('vindices',vindices,'origvertices',vertices,'dindices',dindices,'good',good));
    % dindices and good do have meaning in casenum==3 but we need them only
    % for the viewsurfacedata_replace case, so let's restrict saving them to casenum==2
end
if casenum==3
  set(pobj,'UserData',struct('vindices',vindices,'origvertices',vertices));
end
