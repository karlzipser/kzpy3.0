function viewsurfacedata_quit(wantreport)

% function viewsurfacedata_quit
%
% cleans up program state and closes any
% existing gui window (but not render window).
% this function is useful for when you think the
% program state is stale or has been corrupted
% and you want to start with a clean slate or
% if you want to free memory.

% internal note:
% <wantreport> is a hidden parameter such that
%   <wantreport> is whether to report to the 
%   the command window.  if [] or not supplied,
%   default to 1.

if ~exist('wantreport','var')
  wantreport = [];
end
if isempty(wantreport)
  wantreport = 1;
end

viewsurfacedata_closegui;
viewsurfacedata_disablerender;

% make sure globals are cleared
clear global VS_REFSRF VS_RXYZ VS_RVNUM VS_RCOLORS VS_RNEIGHBORS VS_RISOLATED VS_RFACES;
clear global VS_TARGETSRF VS_TXYZ VS_TVNUM VS_TCOLORS VS_TNEIGHBORS VS_TISOLATED VS_TFACES;
clear global VS_TNORMALS;
clear global VS_SRFTYPE;
clear global VS_R VS_RPTR;
clear global VS_XYZBEGIN VS_XYZEND;
clear global VS_DATA VS_PDATA;
clear global VS_SPECIALBV VS_SPECIALSV VS_SPECIALCVLOW VS_SPECIALCVHIGH;
clear global VS_INTERVAL;
clear global VS_DRANGE VS_CDRANGE;
clear global VS_FIG VS_GUI;
clear global IRT_FACES;
clear global VS_CMAPTYPE;
clear global VS_CMAPNUM;
clear global VS_BEGINVALUE VS_ENDVALUE;
clear global VS_SELECTION;
clear global VS_TR;
clear global VS_OUTLINEC VS_OUTLINEE VS_OUTLINEV;

% report
if wantreport
  fprintf(1,'viewsurfacedata program state successfully cleared.\n');
end
