function f = freadword(fid)

% function f = freadword(fid)
%
% read uchars until 0 is encountered.
% return the string.

f = '';
getc = fread(fid,1,'uchar');
while getc~=0
  f = [f getc];
  getc = fread(fid,1,'uchar');
end
