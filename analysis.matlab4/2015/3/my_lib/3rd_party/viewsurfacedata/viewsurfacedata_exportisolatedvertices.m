function f = viewsurfacedata_exportisolatedvertices

% function f = viewsurfacedata_exportisolatedvertices
%
% returns a vector of 0/1 indicating which vertices in
% the target surface are isolated.

global VS_GUI VS_TISOLATED;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% do it
f = VS_TISOLATED;
