function f = colorname2num(x)

% function f = colorname2num(x)
%
% <x> is a color string (including the added 'gray')
%
% return the 1-indexed number corresponding to <x>.
%
% see also listcolornames.m, colorname2num.m, iscolorname.m.

switch x
case 'red'
  f = 1;
case 'green'
  f = 2;
case 'blue'
  f = 3;
case 'cyan'
  f = 4;
case 'magenta'
  f = 5;
case 'yellow'
  f = 6;
case 'black'
  f = 7;
case 'white'
  f = 8;
case 'gray'
  f = 9;
case 'light gray'
  f = 10;
end
