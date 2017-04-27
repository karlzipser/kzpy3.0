function varargout = viewsurfacedata_constants(varargin)

% function varargout = viewsurfacedata_constants(varargin)
%
% <varargin> = cell vector of field names
%
% <varargout> = cell vector of corresponding values

for p=1:length(varargin)
  switch varargin{p}
  case 'slen'
    varargout{p} = 1/10;
% OLD
%   case 'settingsfile'
%     varargout{p} = '~/.surfpak/settings.mat';  % OBSOLETE
  case 'edgealphav'
    varargout{p} = .3;
  case 'bcolors'  % brain
    varargout{p} = { [1 0 0] [0 1 0] [0 0 1] [0 1 1] [1 0 1] [1 1 0] [0 0 0] [1 1 1] [.5 .5 .5] [.75 .75 .75]};
  case 'scolors'  % selection
% % [0.7250         0    0.4250] [1.0000    0.4000         0] [0.1  0.8  0] [0    0.4500    1.0000]
%red: [1          0.16078431372549                         0]
% blue: [0.368627450980392         0.317647058823529                         1]
    varargout{p} = { [1 0 0] [0 1 0] [0 0 1] [0 1 1] [1 0 1] [1 1 0] [0 0 0] [1 1 1] [.5 .5 .5]};
  case 'ccolors'  % curvature
    varargout{p} = { [1 0 0; .5 0 0] [0 1 0; 0 .5 0] [0 0 1; 0 0 .5] [0 1 1; 0 .5 .5] [1 0 1; .5 0 .5] [1 1 0; .5 .5 0] [.5 .5 .5; 0 0 0] [1 1 1; .5 .5 .5] };
  case 'pcolors'  % partial-volume
    varargout{p} = { [1 0 0] [0 1 0] [0 0 1] [0 1 1] [1 0 1] [1 1 0] [0 0 0] [1 1 1] [.5 .5 .5]};
  case 'icolors'  % intersection
    varargout{p} = { [1 0 0] [0 1 0] [0 0 1] [0 1 1] [1 0 1] [1 1 0] [0 0 0] [1 1 1] [.5 .5 .5]};
  case 'bgcolors'  % background
    varargout{p} = { [1 0 0] [0 1 0] [0 0 1] [0 1 1] [1 0 1] [1 1 0] [0 0 0] [1 1 1] [.5 .5 .5] [.85 .85 .85]};
  case 'colornames'
    varargout{p} = listcolornames;
  case 'colornamesdouble'
    varargout{p} = {'red / dark red' 'green / dark green' 'blue / dark blue' ...
                    'cyan / dark cyan' 'magenta / dark magenta' 'yellow / dark yellow' ...
                    'gray / black' 'white / gray'};
  case 'cmaps'  % default set of colormaps
    varargout{p} = listcolormaps;
  case 'cmapnum'  % default number of colors per colormap
    varargout{p} = 64;
  case 'normalshift'
    varargout{p} = .02;
  case 'figpos'
%    varargout{p} = [.05 .25 .425 .5];
    varargout{p} = [57.6 159.2 489.6 319.2];  % used to be 319.2, 318.2
  case 'guipos'
%    varargout{p} = [.05+.425+.05 .25 .425 .5];
    varargout{p} = [604.8 159.2 489.6 318.2];
  end
end
