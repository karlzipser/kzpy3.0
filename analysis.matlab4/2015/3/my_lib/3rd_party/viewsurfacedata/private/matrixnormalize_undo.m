function f = matrixnormalize_undo(m,minv,maxv,mmin,mmax)

% function f = matrixnormalize_undo(m,minv,maxv,mmin,mmax)
%
% <m> is the result of the call to matrixnormalize
% <minv>,<maxv>,<mmin>,<mmax> as in matrixnormalize
%   note that <mmin> and <mmax> are required in this call.
%
% return the original version of <m>.  but note that if chop was
% used in the original call to matrixnormalize, then obviously
% the exact version of <m> will not be returned.

% check empty case
if isempty(m)
  f = [];
  return;
end

% init f
f = zeros(size(m));

% what cases are weird?
weirdcase = mmin==mmax;
if isscalar(weirdcase)
  weirdcase = repmat(weirdcase,size(m));
end

% deal with weird case
if any(weirdcase)
  f(weirdcase) = subscript(m + (mmin - minv),weirdcase);
end

% deal with normal case
  % want to do: f = (m-minv) .* (mmax-mmin)./(maxv-minv) + mmin
if any(~weirdcase)
  val = (mmax-mmin)./(maxv-minv);
  f(~weirdcase) = subscript(m.*val - (minv.*val - mmin),~weirdcase);  % like this for speed
end
