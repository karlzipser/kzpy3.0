function f = matrixsegment(m,elt)

% function f = matrixsegment(m,elt)
%
% <m> is a vector where <elt> indicates boundaries between
%   segments.  <elt> can occur anywhere.
% <elt> (optional) is the element that separates
%   segments.  if [] or not supplied, default to NaN.
%
% return a 2 x M matrix like
% [A1 B1 C1 ...
%  A2 B2 C2 ...]
% where columns are the starting and ending indices
% of segments.  moreover, these columns are in the
% expected order.

% deal with input
if ~exist('elt','var') | isempty(elt)  % for compatibility with matlab 5
  elt = NaN;
end

% do it
if isnan(elt)
  t1 = find(~isnan(m));
else
  t1 = find(m~=elt);
end
if isempty(t1)
  f = reshape([],[2 0]);
  return;
end
t2 = find(diff(t1)~=1);

cnt = 1;  % plain counter
cnt2 = 1;  % count into t1
f = [];

while 1
  if cnt<=length(t2)
    f = [f [t1(cnt2) t1(t2(cnt))]'];
  else
    f = [f [t1(cnt2) t1(end)]'];
    break;
  end
  cnt2 = t2(cnt)+1;
  cnt = cnt + 1;
end
