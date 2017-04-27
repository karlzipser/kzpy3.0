function f = anycell(c)

% function f = anycell(c)
%
% <c> is a cell matrix of booleans.
%
% return whether any are true.

f = 0;
for p = 1:numel(c)
  f = f || c{p};
end
