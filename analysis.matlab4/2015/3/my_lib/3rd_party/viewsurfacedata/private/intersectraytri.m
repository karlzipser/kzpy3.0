function [ip,cp,cpi] = intersectraytri(rp0,rp1,changed)

% function [ip,cp,cpi] = intersectraytri(rp0,rp1,changed)
%
% rp0: initial ray point, 1 x 3
% rp1: final ray point, 1 x 3
% changed (optional): whether the global IRT_FACES has changed.
%   defaults to 1 to be on the safe side.
%
% globals:
%   IRT_FACES: N (faces) x 3 (vertices per face) x 3 (coordinates per vertex)
%
% return ip as intersection point (1 x 3) associated with closest intersecting face.
% return cp as point on that face closest to ip.
% return cpi as [a b] where a is the face index and b is the
%   vertex index corresponding to cp.
% if there are no intersecting faces, return all outputs as [].
%
% NOTE: this is based on code contained in intersectraytri.cpp!

%%%%%%%%%%%%%%%%%%%%%%%%%

% globals
global IRT_FACES;

persistent IRT_U IRT_V IRT_N IRT_UU IRT_UV IRT_VV IRT_D;
persistent IRT_RP0_OLD IRT_RP0UP IRT_W0 IRT_A;

if ~exist('changed','var')
  changed = 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%% calculations that are independent of the ray

% speed-up number 1
if changed
  IRT_U = squeezeout(IRT_FACES(:,2,:) - IRT_FACES(:,1,:),2);          % N x 3
  IRT_V = squeezeout(IRT_FACES(:,3,:) - IRT_FACES(:,1,:),2);          % N x 3
  IRT_N = cross(IRT_U,IRT_V,2);                                  % N x 3

  % at this point, we should handle degenerate cases, but we don't expect them.

  IRT_UU = dot(IRT_U,IRT_U,2);         % N x 1
  IRT_UV = dot(IRT_U,IRT_V,2);         % N x 1
  IRT_VV = dot(IRT_V,IRT_V,2);         % N x 1

  IRT_D = IRT_UV.^2 - IRT_UU.*IRT_VV;  % N x 1
end

%%%%%%%%%%%%%%%%%%%%%%%%% calculations that are dependent on the ray

numfaces = size(IRT_FACES,1);
rp1up = repmat(rp1,[numfaces 1]);  % N x 3

% speed-up number 2
if changed || ~isequal(rp0,IRT_RP0_OLD)
  IRT_RP0_OLD = rp0;
  IRT_RP0UP = repmat(rp0,[numfaces 1]);              % N x 3
  IRT_W0 = IRT_RP0UP-squeezeout(IRT_FACES(:,1,:),2);      % N x 3
  IRT_A = -dot(IRT_N,IRT_W0,2);                      % N x 1
end

dir = rp1up-IRT_RP0UP;                 % N x 3
b = dot(IRT_N,dir,2);                  % N x 1

% at this point, should check for parallel (disjoint or same plane) cases.
% for speed, let's not do it.

prev = warning('query','MATLAB:divideByZero');
warning('off','MATLAB:divideByZero');

r = IRT_A ./ b;  % N x 1

% at this point, should filter out the "away" cases.  for speed (?), let's not do it.

intersects = IRT_RP0UP + repmat(r,[1 3]).*dir;  % N x 3

w = intersects - squeezeout(IRT_FACES(:,1,:),2);  % N x 3
wu = dot(w,IRT_U,2);                         % N x 1
wv = dot(w,IRT_V,2);                         % N x 1
s = (IRT_UV.*wv - IRT_VV.*wu)./IRT_D;        % N x 1
t = (IRT_UV.*wu - IRT_UU.*wv)./IRT_D;        % N x 1

warning(prev);

intersectfaces = s>=0 & s<=1 & t>=0 & (s+t)<=1;

if any(intersectfaces)
  % find closest face and then corresponding intersection
  [mn,idx] = min(r(intersectfaces));
  closestface = subscript(find(intersectfaces),idx);
  ip = intersects(closestface,:);
  
  % ok, find closest vertex
  vertices = squeezeout(IRT_FACES(closestface,:,:),1);
  [mn,idx] = min(sum((vertices - repmat(ip,[size(vertices,1) 1])).^2,2));
  cp = vertices(idx,:);
  cpi = [closestface idx];
else
  ip = [];
  cp = [];
  cpi = [];
end
