function f = matrixthreshold(m,vlow,vhigh,badlow,badhigh)

% function f = matrixthreshold(m,vlow,vhigh,badlow,badhigh)
%
% <m> = matrix
% <vlow> = value for low threshold
%   if [], don't do
% <vhigh> = value for high threshold
%   if [], don't do
% <badlow> = value for low values thresholded out
%   if [] or not supplied, assume vlow
% <badhigh> = value for high values thresholded out
%   if [] or not supplied, assume vhigh
%
% return <m> with out-of-range values thresholded out.

if ~exist('badlow','var') || isempty(badlow)
  badlow = vlow;
end
if ~exist('badhigh','var') || isempty(badhigh)
  badhigh = vhigh;
end

f = m;

if ~isempty(vlow)
  f(f<vlow) = badlow;
end

if ~isempty(vhigh)
  f(f>vhigh) = badhigh;
end
