function hs = viewsurfacedata_drawaxesdir(hs)

% function hs = viewsurfacedata_drawaxesdir(hs)
%
% in all cases, hs is a set of existing line and text objects to mangle.
% if [], then we make a new set of line and text objects.
%
% draw and return handles to line and text objects.

global VS_TXYZ;

slen = viewsurfacedata_constants('slen');

% handle input
if isempty(hs)
  hs = [plot3empty plot3empty plot3empty text text text];
end

xmin = min(VS_TXYZ(1,:));
xmax = max(VS_TXYZ(1,:));
ymin = min(VS_TXYZ(2,:));
ymax = max(VS_TXYZ(2,:));
zmin = min(VS_TXYZ(3,:));
zmax = max(VS_TXYZ(3,:));

xtarget = xmin + (xmax-xmin)*slen;
ytarget = ymin + (ymax-ymin)*slen;
ztarget = zmin + (zmax-zmin)*slen;

set(hs(1:3),'LineStyle','-', ...
            'Color','red');
set(hs(4:6),'Clipping','off', ...
            'Color',[.5 .5 .5], ...
            'HorizontalAlignment','center');
set(hs(1),'XData',[xmin;xtarget], ...
          'YData',[ymin;ymin], ...
          'ZData',[zmin;zmin]);
set(hs(2),'XData',[xmin;xmin], ...
          'YData',[ymin;ytarget], ...
          'ZData',[zmin;zmin]);
set(hs(3),'XData',[xmin;xmin], ...
          'YData',[ymin;ymin], ...
          'ZData',[zmin;ztarget]);
set(hs(4),'Position',[xtarget ymin zmin], ...
          'String','x');
set(hs(5),'Position',[xmin ytarget zmin], ...
          'String','y');
set(hs(6),'Position',[xmin ymin ztarget], ...
          'String','z');
