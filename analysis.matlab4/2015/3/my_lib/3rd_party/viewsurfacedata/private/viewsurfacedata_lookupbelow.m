function dvalues = viewsurfacedata_lookupbelow(vindices,below,belowp,belowpdir)

% function dvalues = viewsurfacedata_lookupbelow(vindices,below,belowp,belowpdir)
%
% <vindices> is a vector of the vertex indices we want to lookup
% <below> is a vector of surfs indices indicating the order of surfs
%   that are below the current surface.  for example, [5 3 4] means
%   that the bottommost layer (excluding the brain surface) has surfs
%   index 5, the next layer has index 3, the next layer has index 4,
%   and the next layer after that is the current surface itself.
% <belowp> are the corresponding p-values.  can be a scalar.
% <belowpdir> are the corresponding p-direction values.  can be a scalar.
%
% return a vector of the same length as <vindices> populated with the correct data values!

global VS_DATA VS_PDATA VS_TCOLORS VS_SPECIALSV VS_SPECIALCVLOW VS_SPECIALCVHIGH VS_SPECIALBV VS_SELECTION;

% deal with input
if length(belowp)==1
  belowp = repmat(belowp,1,length(below));
end
if length(belowpdir)==1
  belowpdir = repmat(belowpdir,1,length(below));
end

% initialize
dvalues = NaN*zeros(1,length(vindices));

% logical indices of things left to populate
toprocess = logical(ones(1,length(vindices)));

% go through below surfaces backwards!
for p=length(below):-1:1
  % if we're done already, get out early!
  if ~any(toprocess)
    break;
  end
  % init
  idxtoupdate = [];
  % <sval> is the surface we are now looking at
  sval = below(p);
  % define (the reason this comes at a weird location is that it is possible
  % for this function to be called before the gui is initialized.  those cases
  % are the initialization cases and should have below==[] so we won't ever get here anyway.)
  global VS_GUI;
  handles = guidata(VS_GUI);
  % handle regular surface
  if ismember(sval,handles.rsurf)
    [dindices,good] = viewsurfacedata_lookup(sval-1,vindices(toprocess));
    if ~isempty(VS_PDATA{sval-1})
      switch belowpdir(p)
      case 1
        found = find(VS_PDATA{sval-1}(dindices) < belowp(p));
      case 2
        found = find(VS_PDATA{sval-1}(dindices) <= belowp(p));
      case 3
        found = find(VS_PDATA{sval-1}(dindices) > belowp(p));
      case 4
        found = find(VS_PDATA{sval-1}(dindices) >= belowp(p));
      end
      idxtoupdate = subscript(subscript(find(toprocess),good),found);  % indices that we are updating
      dvalues(idxtoupdate) = VS_DATA{sval-1}(dindices(found));
    else
      idxtoupdate = subscript(find(toprocess),good);  % indices that we are updating
      dvalues(idxtoupdate) = VS_DATA{sval-1}(dindices);
    end
  end
  % handle selection surface (under the condition that it is not in outline mode)
  if ismember(sval,handles.ssurf) && handles.surfacerecord{sval}.boundary~=3
    found = find(ismember(vindices(toprocess),VS_SELECTION{sval}));
    idxtoupdate = subscript(find(toprocess),found);  % indices that we are updating
    dvalues(idxtoupdate) = VS_SPECIALSV{sval-handles.ssurf(1)+1};
  end
  % handle curvature surface
  if ismember(sval,handles.csurf)
    switch belowpdir(p)
    case 1
      found = find(VS_TCOLORS(vindices(toprocess)) < belowp(p));
    case 2
      found = find(VS_TCOLORS(vindices(toprocess)) <= belowp(p));
    case 3
      found = find(VS_TCOLORS(vindices(toprocess)) > belowp(p));
    case 4
      found = find(VS_TCOLORS(vindices(toprocess)) >= belowp(p));
    end
    idxtoupdate = subscript(find(toprocess),found);  % indices that we are updating
    idxfromdata = subscript(vindices(toprocess),found);  % actual vertex indices that we are updating
    dvalues(idxtoupdate) = normalizerange(VS_TCOLORS(idxfromdata),VS_SPECIALCVLOW,VS_SPECIALCVHIGH,0,1,1);  % the chop probably isn't necessary
  end
  % ok, mark those we updated as no longer needed
  toprocess(idxtoupdate) = false;
end

% any remaining get the brain color
dvalues(toprocess) = VS_SPECIALBV;

% sanity check
%assert(~any(isnan(dvalues)));
