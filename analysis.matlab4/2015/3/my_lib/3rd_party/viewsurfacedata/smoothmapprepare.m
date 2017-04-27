function [weight,nbr] = smoothmapprepare(srf)

% function [weight,nbr] = smoothmapprepare(srf)
%
% <srf> is like what is passed to viewsurfacedata.
%   it should be the fiducial surface, as it is 
%   guaranteed to have accurate neighbors information.
%
% using the FEM method, return the weighting and neighbors information
% used in surface map smoothing (see smoothmap.m).  these bits of
% information are invariant for a given surface, so you can save them
% and use them as needed.
%
% this function is on based diffusion_smooth.m, for which the
% following applies:
%############################################################################
% COPYRIGHT: Copyright 2000 M.K. Chung, K.J. Worsley, J. Taylor, J.O. Ramsay 
%            Department of Mathematics and Statistics,
%            McConnell Brain Imaging Center, 
%            Montreal Neurological Institute,
%            Department of Psychology,
%            McGill University, Montreal, Quebec, Canada. 
%            Corresponding Address: chung@math.mcgill.ca
%
%            Permission to use, copy, modify, and distribute this
%            software and its documentation for any purpose and without
%            fee is hereby granted, provided that the above copyright
%            notice appear in all copies.  The author and McGill University
%            make no representations about the suitability of this
%            software for any purpose.  It is provided "as is" without
%            express or implied warranty.
%############################################################################

% do it
fprintf(1,'step 1: loading surface...\n');
if ischar(srf)
  temp = load(srf,'vertices','faces');
  coord = temp.vertices;
  tri = temp.faces;
else
  [coord,tri] = loadvtk(srf{:});  % coordinates (4 x V) and faces (F x 3)
end
coord = coord(1:3,:);  % coordinates (3 x V)
n_points = size(coord,2);  % number of vertices
fprintf(1,'step 2: finding vertex neighbors...\n');
nbr = facestoneighbors2(tri,n_points);  % neighbors (V x N); see facestoneighbors2.m
fprintf(1,'step 3: calculating weighting...\n');
weight = femweight(coord,nbr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function f = femweight(coord,nbr)

% calc
n_points = size(nbr,1);
numnbr = size(nbr,2) - sum(nbr==0,2);  % number of neighbors per vertex (V x 1)
dnumnbr = unique(numnbr');  % vector of distinct number of neighbors (1 x D)
maxnbr = max(dnumnbr);  % maximum number of neighbors

% do it
f = zeros(n_points,maxnbr);
for p=1:n_points
  if mod(p,1000)==0, fprintf(1,'.');, end
  n_nbr = numnbr(p);
  nbr_list = nbr(p,1:n_nbr);
  coord_nbr = coord(:,nbr_list) - kron(ones(1,n_nbr),coord(:,p));
  % ok, loop
  cotan = zeros(1,n_nbr);
  area = 0;
  y = 1:n_nbr;
  x = mod2(y-1,n_nbr);
  z = mod2(y+1,n_nbr);
  for q=1:length(cotan)
    coordx = coord_nbr(:,x(q))-coord_nbr(:,y(q));
    coordz = coord_nbr(:,z(q))-coord_nbr(:,y(q));
    crossx = cross(coord_nbr(:,x(q)),coordx);
    crossz = cross(coord_nbr(:,z(q)),coordz);
    areax = sqrt(crossx'*crossx)/2;
    areaz = sqrt(crossz'*crossz)/2;
    dotx = coord_nbr(:,x(q))'*coordx;
    dotz = coord_nbr(:,z(q))'*coordz;
    cotan(q) = dotx/(2*areax) + dotz/(2*areaz);
    area = area + areaz;
  end
  f(p,1:n_nbr) = cotan/area;
end
fprintf(1,'\n');






% FIX?:

% <parametric> is "1 if local quadratic parameterization is used. 
%                  This method is analytically more accurate."
%              is "0 if FEM method is used."

% switch parametric
% case 0
% case 1
%   assert(0);  % FIXME
%   fig = figure('Visible','off');
%   h = trisurf(tri,coord(1,:),coord(2,:),coord(3,:));
%   normal = get(h,'VertexNormals')';  % normals (3 x V)
%   delete(fig);
%   weight = parametricweight(coord,nbr,normal);
% end

% function f = parametricweight(coord,nbr,normal)
% 
% % calc
% n_points = size(nbr,1);
% numnbr = size(nbr,2) - sum(nbr==0,2);  % number of neighbors per vertex (V x 1)
% dnumnbr = unique(numnbr');  % vector of distinct number of neighbors (1 x D)
% maxnbr = max(dnumnbr);  % maximum number of neighbors
% 
% % do it
% f = zeros(n_points,maxnbr);
% for p=1:n_points
%   if mod(p,1000)==0, fprintf(1,'.');, end
%   n_nbr = numnbr(p);
%   nbr_list=nbr(p,1:n_nbr);
%   %translate to the origin.
%   coord_nbr = coord(:,nbr_list)-kron(ones(1,n_nbr),coord(:,p));
%   % Rotation by the Eulerian Angles   
%   % Q1 rotation to align to x-axis.
%   % Q2 rotation to align to z-axis.
%   
%   n=normal(:,p);
%   deno= sqrt(n(1)^2+n(2)^2);   
%   if deno < 0.1
%      Q=eye(3);
%   else
%      Q1 = [[n(1)/deno,n(2)/deno,0];
%         [-n(2)/deno,n(1)/deno,0];
%         [0,     0,      1]];
%      Q2 = [[n(3),0, -deno];
%         [0,      1,      0];
%         [deno,0,n(3)]];
%      Q=Q2*Q1;
%   end;
%   %if Q*n = (0,0,1), the algorithm is correct
%   
%   rot_coord_nbr = Q*coord_nbr;
%   
%   % beta = (X'X)^(-1)X'z
%   
%   z = [rot_coord_nbr(3,:)]';
%   x = [rot_coord_nbr(1,:)]';
%   y = [rot_coord_nbr(2,:)]';
%   
%   % X : design matrix 
%   X=[x, y, x.^2, 2*x.*y, y.^2]; %Xbeta =z
%   beta = (X\z)';
%   % Conformal Coordinate Transform
%   
%   temp2 = beta(1)*beta(2);
%   temp1=[[1+beta(1)^2,temp2];
%      [temp2,1+beta(2)^2]];
%   ghalf=sqrtm(temp1);
%   temp=ghalf*[x';y'];
% 
%   x = temp(1,:);
%   y = temp(2,:);
%   X_activation = [x, y, 1/2*x.^2, x.*y, 1/2*y.^2];
%   gamma = pinv(X_activation);
%   f(p,1:n_nbr)=gamma(3,:)+gamma(5,:); 
% end
% fprintf(1,'\n');
