function f = writesnapshot(fig,dir,prefix,numzeros,suffix,method,optstr)

% function f = writesnapshot(fig,dir,prefix,numzeros,suffix,method,optstr)
%
% <fig> (optional) is a figure handle.
%   if [] or not supplied, default to gcf.
% <dir> (optional) is the directory to save into
%   if [] or not supplied, default to '.'.
% <prefix> (optional) is a prefix for the filename
%   if [] or not supplied, default to 'image'.
% <numzeros> (optional) is how many zeros to use for numbering
%   if [] or not supplied, default to 3.
% <suffix> (optional) must be 'png'
%   if [] or not supplied, default to 'png'.
% <method> (optional) is 1 means use getframe; 2 means use print
%   if [] or not supplied, default to 1.
% <optstr> (optional) is an optional print string.
%
% return the location of the file written.
%
% TODO: make more general?? (different image formats, different output sizes)
% REVISIT

if ~exist('fig','var') || isempty(fig)
  fig = gcf;
end
if ~exist('dir','var') || isempty(dir)
  dir = '.';
end
if ~exist('prefix','var') || isempty(prefix)
  prefix = 'image';
end
if ~exist('numzeros','var') || isempty(numzeros)
  numzeros = 3;
end
if ~exist('suffix','var') || isempty(suffix)
  suffix = 'png';
end
if ~exist('method','var') || isempty(method)
  method = 1;
end
if ~exist('optstr','var') || isempty(optstr)
  optstr = '';
end

%assert(isequal(suffix,'tif'));

cnt = 0;
f = [dir,filesep,prefix,padzeros(cnt,numzeros),'.',suffix];
while fileexists(f)
  cnt = cnt + 1;
  f = [dir,filesep,prefix,padzeros(cnt,numzeros),'.',suffix];
end

switch method
case 1
  fr = getframe(fig);
  imwrite(fr.cdata,f);
case 2
%   % removed opengl, added zbuffer
%   % remove '-zbuffer'
  print(fig,'-dpng','-r0',optstr,f);
end
