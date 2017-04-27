function f = findfaceintersections(XYZ,faces,dim,slices,neighbors,limits)

% function f = findfaceintersections(XYZ,faces,dim,slices,neighbors,limits)
%
% <XYZ> is 4 x V
% <faces> is F x 3
% <dim> is 1 | 2 | 3
% <slices> is a vector of coordinate(s) of the plane(s)
%   in dimension <dim>
% <neighbors> (optional) is the output from facestoneighbors.m.
%   if [] or not supplied, we call the function (which slows us down).
% <limits> (optional) indicates the x-, y-, and z-dimension
%   limits on intersections.  if [] or not supplied, default to
%   [-Inf Inf; -Inf Inf; -Inf Inf] (which imposes no limits).
%
% calculate a matrix that is composed of
% vertical concatenations of things like:
%     <interp1> <a1> <b1>
%     <interp2> <a2> <b2>
%     ...
%     <interpn> <an> <bn>
%     NaN          1    1
% each thing represents one contiguous line, and things
% consist of zero or more interpX entries.
% the row with interp1 might equal the row with interpn,
%   in which case that represents a circular case.
% the second and third columns refer to vertex indices.

% deal with input
if ~exist('neighbors','var') || isempty(neighbors)
  neighbors = facestoneighbors(faces,size(XYZ,2));
end
if ~exist('limits','var') || isempty(limits)
  limits = repmat([-Inf Inf],[3 1]);
end

%%%%% STEP 1.

% calculate <fi> which is a F x 3 matrix.
%   a row with all NaNs means that it doesn't intersect any plane
%   a row = [a b c] means a is the scalar from vertex 1 to 2
%                         b is the scalar from vertex 2 to 3
%                         c is the scalar from vertex 3 to 1
%     where intersections occur only for scalars in [0,1]
%       and where Inf means no intersection

% init
fi = repmat(NaN,size(faces));

% do it
  prev = warning('query','MATLAB:divideByZero');
  warning('off','MATLAB:divideByZero');
for p=1:length(slices)
  slice = slices(p);

  oneside = XYZ(dim,:) > slice;  % 1 x V, whether to one side of plane
  oneside1 = oneside(faces(:,1));
  oneside2 = oneside(faces(:,2));
  oneside3 = oneside(faces(:,3));
  ifaces = ~(oneside1 & oneside2 & oneside3) & (oneside1 | oneside2 | oneside3);  % 1 x F, logicals of intersecting faces
  
  p1 = XYZ(1:3,faces(ifaces,1));  % 3 x F2, coordinates of first vertex
  p2 = XYZ(1:3,faces(ifaces,2));  % 3 x F2, coordinates of second vertex
  p3 = XYZ(1:3,faces(ifaces,3));  % 3 x F2, coordinates of third vertex
  
  good1 = p1(1,:) >= limits(1,1) & p1(1,:) <= limits(1,2) & ...
          p1(2,:) >= limits(2,1) & p1(2,:) <= limits(2,2) & ...
          p1(3,:) >= limits(3,1) & p1(3,:) <= limits(3,2);
  good2 = p2(1,:) >= limits(1,1) & p2(1,:) <= limits(1,2) & ...
          p2(2,:) >= limits(2,1) & p2(2,:) <= limits(2,2) & ...
          p2(3,:) >= limits(3,1) & p2(3,:) <= limits(3,2);
  good3 = p3(1,:) >= limits(1,1) & p3(1,:) <= limits(1,2) & ...
          p3(2,:) >= limits(2,1) & p3(2,:) <= limits(2,2) & ...
          p3(3,:) >= limits(3,1) & p3(3,:) <= limits(3,2);
  good = good1 | good2 | good3;

  s1 = (slice - p1(dim,good))./(p2(dim,good)-p1(dim,good));  % 1 x F2, scalar interp from first to second
  s2 = (slice - p2(dim,good))./(p3(dim,good)-p2(dim,good));  % 1 x F2, scalar interp from second to third
  s3 = (slice - p3(dim,good))./(p1(dim,good)-p3(dim,good));  % 1 x F2, scalar interp from third to first
    
  fi(subscript(find(ifaces),good),:) = [s1; s2; s3]';
end
  warning(prev);

%%%%% STEP 2.

% continue
s1 = fi(:,1)>=0 & fi(:,1)<=1;  % 1 x J, indices of faces whose first segment passed
s2 = fi(:,2)>=0 & fi(:,2)<=1;  % 1 x K, indices of faces whose second segment passed
s3 = fi(:,3)>=0 & fi(:,3)<=1;  % 1 x L, indices of faces whose third segment passed
valid = s1 | s2 | s3;
  % make sure that weird cases don't actually happen!
  % FIXME: this relies on there always being residual decimals.
assert(~any(flatten(fi(valid,:) == 0 | fi(valid,:) == 1)));
connect1 = faces(valid,1);  % M x 1, first vertex
connect2 = faces(valid,2);  % M x 1, second vertex
connect3 = faces(valid,3);  % M x 1, third vertex
sc1 = fi(valid,1);  % M x 1, scalar factor
sc2 = fi(valid,2);  % M x 1, scalar factor
sc3 = fi(valid,3);  % M x 1, scalar factor
case1 = sc1 < 0 | sc1 > 1;
case2 = sc2 < 0 | sc2 > 1;
case3 = sc3 < 0 | sc3 > 1;
in1 = [sc2(case1)'; connect2(case1)'; connect3(case1)'; ...
       sc3(case1)'; connect3(case1)'; connect1(case1)'];
in2 = [sc1(case2)'; connect1(case2)'; connect2(case2)'; ...
       sc3(case2)'; connect3(case2)'; connect1(case2)'];
in3 = [sc1(case3)'; connect1(case3)'; connect2(case3)'; ...
       sc2(case3)'; connect2(case3)'; connect3(case3)'];

% whew, now we're ready to pass off to perl
%
% to the temp file we write a matrix where columns are like:
%   ainterp
%   avertexnum1
%   avertexnum2
%   binterp
%   bvertexnum1
%   bvertexnum2
% the existence of a column indicates that there is an edge between the
% positions represented by a and b.
%
% we expect from the temp file, the following:
%   repeat for all found faces:
%     <number of vertices in this face>
%     for each vertex:
%       interp
%       vertexnum1
%       vertexnum2
%
% circular cases are indicated by the first vertex in a face being identical to the last.
%
% now let's do it!
%
tempfile = tempname;
byteorder = perl(strrep(which('findfaceintersections'), ...
                              'findfaceintersections.m', ...
                              'getbyteorder.pl'));
assert(isequal(byteorder,'l'));
savebinary(tempfile,'double',[in1 in2 in3],0);
perl(strrep(which('findfaceintersections'), ...
                  'findfaceintersections.m', ...
                  'findfaceintersections_helper.pl'),tempfile);
out = loadbinary(tempfile,'double',[0 1]);
%delete(tempfile);  % FIXME: should we delete here?

% post-process perl output
f = [];
outcnt = 1;
while outcnt <= length(out)
  numvertices = out(outcnt);
  temp = reshape(out(outcnt+1:outcnt+3*numvertices),[3 numvertices])';  % N x 3
    % ok.  now we remove any vertex that is on an edge which seems
    % to not exist in the target surface.  this is expected in the
    % flattened target surface case where faces and vertices can just disappear.
    % note that the following mechanism subsumes the previous check for edges that
    % involve a vertex that is isolated (i.e. not involved in any face) in the target
    % surface.  and furthermore note that the following mechanism catches cases that
    % the previous check wouldn't have caught.  so, the following mechanism should 
    % catch all desired cases, i think.  (TODO: think about this)
  bad = [];
  for p=1:size(temp,1)
    if ~ismember(temp(p,3),neighbors{temp(p,2)})
      bad = [bad p];
    end
  end
    % continue
  temp(bad,:) = repmat([NaN 1 1],[length(bad) 1]);  % shove in [NaN 1 1] wherever necessary
  f = [f;
       temp;
       NaN 1 1];
  outcnt = outcnt+3*numvertices+1;
end
