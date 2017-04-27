function f = viewsurfacedata_importrois(svgfile,colors,labels,M)

% function f = viewsurfacedata_importrois(svgfile,colors,labels,M)
%
% <svgfile> is the .svg file
% <colors> is a cell vector of hexadecimal colors
% <labels> is a cell vector of strings to be used as field names.
%   should be the same length as <colors>.
% <M> is a 3x3 transformation matrix that maps coordinates in the
%   .svg file to coordinates in the surface.
%
% return a struct with field names as in <labels> and with field values
% as logical vectors indicating the ROI assignments.

% NOTE: based on selectionpolygon_Callback

global VS_TXYZ PIP_VERTICES VS_TISOLATED VS_GUI;

% check
if ~ishandle(VS_GUI)
  fprintf(1,'error: no currently running instance of viewsurfacedata detected.\n');
  return;
end

% setup the point-in-polygon stuff
PIP_VERTICES = VS_TXYZ(1:2,:)';  % consider the first two coordinates of the target surface

% read .svg file
svg0 = catcell(2,loadtext(svgfile));
matches = regexp(svg0,'poly.+?fill="#(......)".+?points="(.+?)"','tokens');

% loop over matches (each match is a polygon)
clear f;
for p=1:length(matches)

  % extract the color of the polygon (1 x 6 char)
  color0 = matches{p}{1};

  % which roi index is this?
  ix = find(ismember(colors,color0));
  assert(length(ix)==1);
  
  % extract the x-y coordinates of the polygon (2 x V)
  coord0 = reshape(sscanf(strrep(matches{p}{2},',',' '),'%f'),2,[]);
  
  % transform coordinates to flat coordinates (2 x U)
  flatcoord0 = subscript(M*[coord0; ones(1,size(coord0,2))],{1:2 ':'});

  % which vertices are inside the polygon?
  vertices = pointinpolygon(flatcoord0')';  % logical 1 x N

  % ignore isolated vertices
  vertices = vertices & ~VS_TISOLATED;

  % record
  f.(labels{ix}) = vertices;

end
