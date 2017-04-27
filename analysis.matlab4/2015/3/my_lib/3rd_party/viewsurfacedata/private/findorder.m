function idx = findorder(x,y)

% function idx = findorder(x,y)
%
% <x> is a vector of unique numbers
% <y> is a vector of unique numbers subsuming x
%
% we want to know the order of the elements
% <x>, as specified by <y>.  return a vector
% indicating the indices of the elements of
% <x> when ordered.

order = zeros(1,length(x));
for p=1:length(x)
  order(p) = firstel(find(y==x(p)));
end
[order,idx] = sort(order);
