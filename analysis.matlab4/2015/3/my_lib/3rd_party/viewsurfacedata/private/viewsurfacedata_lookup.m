function [dindices,good] = viewsurfacedata_lookup(p,vindices,dataselect)

% function [dindices,good] = viewsurfacedata_lookup(p,vindices,dataselect)
%
% <p> is an index into data.  -1 is a special case and means to
%   simulate a full epi volume case using the transformation
%   assocated with the first data volume.
% <vindices> is a vector of vertex indices, possibly a subset
%   of all vertices.  if -1, we use all vertices.
% <dataselect> (optional) is a matrix the same size as data with 0s and 1s.
%   if -1, this in effect means there is no <dataselect>.  note that
%   if <p> corresponds to a vector data case, then <dataselect>
%   is irrelevant and therefore ignored.
%
% basically, return a vector of data indices which correspond to the 
% vertex indices which lie inside valid data regions.  if <dataselect> is
% provided, we use that as a specification of the valid data region.
% if <dataselect> is not provided, we choose the region to be the entire
% data matrix (in the non vector data case) or the non-NaN
% regions of the data vector (in the vector data case).  the
% vector of data indices returned correspond to those vertices that
% lie within the region.  we return <good>, which is a vector
% of INDICES of <vindices> corresponding to those vindices
% which lie within the region.

global VS_DATA VS_R VS_RPTR;

if ~exist('dataselect','var')
  dataselect = -1;
end

if p~=-1 && isempty(VS_RPTR{p})
  if isequal(vindices,-1)
    [dindices,good] = viewsurfacedata_access(p);
  else
    dindices = intersect(viewsurfacedata_access(p),vindices);
    good = find(ismember(vindices,dindices));  % weird but important
  end
  assert(isequal(dataselect,-1));  % sanity check
else
  if p==-1
    rnd = VS_R{VS_RPTR{1}}.rnd;  % this relies on fact that .rnd is unaffected by the <xyzbegin> stuff!
    xyzbegin = VS_R{VS_RPTR{1}}.xyzbegin;
    xyzend = VS_R{VS_RPTR{1}}.xyzend;
    xyzsize = VS_R{VS_RPTR{1}}.xyzsize;
  else
    rnd = VS_R{VS_RPTR{p}}.rnd;
    xyzbegin = VS_R{VS_RPTR{p}}.xyzbegin;
    xyzend = VS_R{VS_RPTR{p}}.xyzend;
    xyzsize = VS_R{VS_RPTR{p}}.xyzsize;
  end

  if isequal(vindices,-1)
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
  else
    good = find(rnd(1,vindices) >= xyzbegin(1) & rnd(1,vindices) <= xyzend(1) & ...
                rnd(2,vindices) >= xyzbegin(2) & rnd(2,vindices) <= xyzend(2) & ...
                rnd(3,vindices) >= xyzbegin(3) & rnd(3,vindices) <= xyzend(3));
    if isempty(good)
      dindices = [];
    else
      dindices = sub2ind(xyzsize,rnd(1,vindices(good))-xyzbegin(1)+1,...
                                 rnd(2,vindices(good))-xyzbegin(2)+1,...
                                 rnd(3,vindices(good))-xyzbegin(3)+1);
    end
  end

  % if dataselect was provided, do further pruning
  if ~isequal(dataselect,-1)
    ds = logical(dataselect(dindices));
    dindices = dindices(ds);
    good = good(ds);
  end
end
