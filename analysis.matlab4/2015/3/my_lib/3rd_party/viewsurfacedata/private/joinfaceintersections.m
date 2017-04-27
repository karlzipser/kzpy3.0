function [faces,vertices,fvad] = joinfaceintersections(XYZ,fi,normals,offset)

% function [faces,vertices,fvad] = joinfaceintersections(XYZ,fi,normals,offset)
%
% <XYZ> is 4 x V
% <fi> is the output from findfaceintersections.m
% <normals> (optional) is 4 x V.
%   if [] or not supplied, do not use normals.
% <offset> (optional) is a scalar.
%   if [] or not supplied, default to 0.
%
% return information for a patch object.

% deal with input
if ~exist('normals','var')
  normals = [];
end
if ~exist('offset','var') || isempty(offset)
  offset = 0;
end

% init
faces = [];
vertices = [];
fvad = [];
vertexcnt = 0;

% get out
if isempty(fi)
  return;
end

% prepare
v1 = XYZ(1:3,fi(:,2));  % 3 x N
v2 = XYZ(1:3,fi(:,3));  % 3 x N
if ~isempty(normals) && offset~=0
  v1 = v1 + offset*normals(1:3,fi(:,2));  % 3 x N
  v2 = v2 + offset*normals(1:3,fi(:,3));  % 3 x N
end
v = v1 + (v2-v1).*repmat(fi(:,1)',[3 1]);  % 3 x N

% do it (mirrored in vsd_drawoutline!)
for p = matrixsegment(v(1,:))
  xbegin = p(1);
  xend = p(2);
  % check for circular case
  if isequal(v(:,xbegin),v(:,xend))
    iscircular = 1;
    xend = xend-1;
  else
    iscircular = 0;
  end
  % calc
  numvertices = xend-xbegin+1;  % number of unique vertices
  % define vertices
  vertices = [vertices; v(:,xbegin:xend)'];  % add <numvertices> x 3 chunk
  % define faces; we repeat the last vertex (so like [1 2 3 3 3 3 3 ...].  does this cause a slowdown???
  if ~isempty(faces)
    if size(faces,2) < numvertices  % enlarge faces if necessary
      faces = [faces repmat(faces(:,end),[1 numvertices-size(faces,2)])];
    end
    newface = placematrix(repmat(vertexcnt+numvertices,[1 size(faces,2)]),vertexcnt+1:vertexcnt+numvertices,[]);
% OLD:
%     newface = matrixpad2(vertexcnt+1:vertexcnt+numvertices, ...
%                          [1 size(faces,2)],'center',vertexcnt+numvertices);  % FIXME: shouldn't this be 'first'?????
% FIXME: IS THE CENTERING RIGHT IN PLACEMATRIX CALL ABOVE?
  else
    newface = vertexcnt+1:vertexcnt+numvertices;
  end
  faces = [faces; newface];
  % define fvad
  newfvad = ones(numvertices,1);  % numvertices x 1
  if ~iscircular
    % the last edge is the one that feeds into the first vertex.
    % so set the alpha of the first vertex to 0.
    % the MATLAB documentation is wrong!!  REVISIT if new MATLAB version?
    newfvad(1) = 0;
  end
  fvad = [fvad; newfvad];
  % increment
  vertexcnt = vertexcnt+numvertices;
end
