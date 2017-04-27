function maps = smoothmap(maps,fwhm,iterations,weight,nbr)

% function maps = smoothmap(maps,fwhm,iterations,weight,nbr)
%
% <maps> is a 2D matrix whose rows are surface map data.
%   all elements must be real numbers (no NaN/Inf).
%   (imaginary numbers should work in theory, i wonder why it doesn't. FIXME)
% <fwhm> is "FWHM of the corresponding Gaussian kernel smoothing in mm"
% <iterations> is the number of iterations to do
% <weight>,<nbr> are the outputs of smoothmapprepare.m
%
% smooth each row of <maps> and return the result.
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

% check
assert(size(weight,1)==size(nbr,1) && size(weight,1)==size(maps,2));

% calc
n_points = size(weight,1);
numnbr = size(nbr,2) - sum(nbr==0,2);  % number of neighbors per vertex (V x 1)
dnumnbr = unique(numnbr');  % vector of distinct number of neighbors (1 x D)
maxnbr = max(dnumnbr);  % maximum number of neighbors

% do it
delta_time = fwhm^2/(16*iterations*log(2));
parfor p=1:size(maps,1)
  fprintf(1,'smoothing map %d\n',p);
  map = maps(p,:);
  df = zeros(n_points,maxnbr);
  minf = zeros(1,n_points);
  maxf = zeros(1,n_points);
  for q=1:iterations
     fprintf(1,'.');
     df(:) = 0;  % like this for spped
     minf(:) = 0;
     maxf(:) = 0;
     for r=1:length(dnumnbr)
       n_nbr = dnumnbr(r);
       idx = numnbr==n_nbr;
       temp = map(nbr(idx,1:n_nbr));
       df(idx,1:n_nbr) = temp - repmat(map(idx)',[1 n_nbr]);
       minf(idx) = min(temp');
       maxf(idx) = max(temp');
     end
     map = map + delta_time*sum(weight.*df,2)';
     map = max(min(map,maxf),minf);
     % should we check and report under/overflow?
  end
  maps(p,:) = map;
  fprintf(1,'\n');
end
