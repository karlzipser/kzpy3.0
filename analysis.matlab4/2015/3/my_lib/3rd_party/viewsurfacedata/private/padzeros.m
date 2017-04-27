function f = padzeros(x,y)

% function f = padzeros(x,y)
%
% pad the number x with leading 0s so as to achieve y digits.
% if x has more than y digits, returns 'x'.

[f,err] = sprintf(['%.',num2str(y),'d'],x);
assert(isempty(err));
