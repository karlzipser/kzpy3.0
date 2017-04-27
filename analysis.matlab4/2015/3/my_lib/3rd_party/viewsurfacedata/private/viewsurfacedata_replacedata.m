function viewsurfacedata_replacedata(pobj,p)

% function viewsurfacedata_replacedata(pobj,p)
%
% <pobj> is existing surface handle.
% <p> is the number of this regular surface.
%
% replace the FaceVertexCData with the appropriate
% values pulled from the current state of VS_DATA.

global VS_DATA;

ud = get(pobj,'UserData');
vertexdata = get(pobj,'FaceVertexCData');
vertexdata(ud.good) = VS_DATA{p}(ud.dindices);
set(pobj,'FaceVertexCData',vertexdata);
