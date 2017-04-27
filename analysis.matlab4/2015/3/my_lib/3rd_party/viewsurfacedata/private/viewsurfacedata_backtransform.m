function f = viewsurfacedata_backtransform(p,dindices)

% function f = viewsurfacedata_backtransform(p,dindices)
%
% <p> is an index into data
% <dindices> (optional) are data indices.
%   if [] or not supplied, we use all.
%
% return the backtransformed data values.

global VS_DATA VS_CDRANGE VS_DRANGE;

if ~exist('dindices','var')
  dindices = [];
end

if isempty(dindices)
  f = matrixnormalize_undo(VS_DATA{p},VS_CDRANGE{p}(1),VS_CDRANGE{p}(2),VS_DRANGE{p}(1),VS_DRANGE{p}(2));
else
  f = matrixnormalize_undo(VS_DATA{p}(dindices),VS_CDRANGE{p}(1),VS_CDRANGE{p}(2),VS_DRANGE{p}(1),VS_DRANGE{p}(2));
end
