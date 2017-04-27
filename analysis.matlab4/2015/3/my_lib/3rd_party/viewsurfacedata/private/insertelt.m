function f = insertelt(m,pos,x)

% function f = insertelt(m,pos,x)
%
% <m> is a vector
% <pos> is position within <m> in [1,Inf)
% <x> is a thing (can be a vector)
%
% return <m> but with <x> inserted at position <pos>.
% we zero-pad if necessary!

f = m;
f(pos+length(x):end+length(x)) = f(pos:end);
%if iscell(f)
%  f{pos:pos+length(x)-1} = x;
%else
  f(pos:pos+length(x)-1) = x;
%end
