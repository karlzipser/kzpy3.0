function f = deleteelt(m,pos)

% function f = deleteelt(m,pos)
%
% <m> is a vector or cell vector
% <pos> is position within <m> in [1,Inf)
%
% return <m> but with the element at position <pos> removed.
%
% for vectors, this is equivalent to m(pos) = [].
% thus, this is mainly useful for cell vectors.

f = m;
if pos <= length(m)
  f(pos:end-1) = f(pos+1:end);
  f(end) = [];
end
