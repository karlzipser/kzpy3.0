function [dindices,vindices] = viewsurfacedata_access(p)

% function [dindices,vindices] = viewsurfacedata_access(p)
%
% <p> is an index into data.
% 
% return data and vertex indices (as vectors).
%
% note that this function is a little peculiar:
% in the vector case, this does a quick calculation.
% in the normal case, this does a quick lookup.

global VS_R VS_RPTR VS_DATA;

if isempty(VS_RPTR{p})
  dindices = find(~isnan(VS_DATA{p}));
  vindices = dindices;
else
  dindices = VS_R{VS_RPTR{p}}.indices;
  vindices = VS_R{VS_RPTR{p}}.valid;
end
