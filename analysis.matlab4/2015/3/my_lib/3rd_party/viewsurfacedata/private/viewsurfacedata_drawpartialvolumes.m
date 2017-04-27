function lineobjs = viewsurfacedata_drawpartialvolumes(fig,ax,pthresh,pthreshdir)

% function lineobjs = viewsurfacedata_drawpartialvolumes(fig,ax,pthresh,pthreshdir)
%
% <fig> is the handle of the figure window in which
%   to draw the line objects.
% <ax> is the axes within that figure window to draw on.
% <pthresh>,<pthreshdir> are as usual.
%
% return a vector of newly created line object handles.
% the projection type must be perspective!

global VS_R VS_RPTR VS_TXYZ VS_TNORMALS VS_PDATA VS_TISOLATED;

% internal constants
movetocamera = 3/100;  % FIXME

% sanity check
assert(~isempty(VS_RPTR{1}));
assert(isequal(camproj(ax),'perspective'));

% do we have the necessary information?
if ~isfield(VS_R{VS_RPTR{1}},'pvv')
  fprintf(1,'calculating partial-volume information...please wait...\n');
  [VS_R{VS_RPTR{1}}.pvv, ...
   VS_R{VS_RPTR{1}}.pvv1, ...
   VS_R{VS_RPTR{1}}.pvv2, ...
   VS_R{VS_RPTR{1}}.pvvvoxel] = viewsurfacedata_pvolumes;
  fprintf(1,'done!\n');
end

% deal with p-thresholding
if ~isempty(VS_PDATA{1})
  switch pthreshdir
  case 1
    good = VS_PDATA{1}(VS_R{VS_RPTR{1}}.pvvvoxel) < pthresh;  % 1 x K
  case 2
    good = VS_PDATA{1}(VS_R{VS_RPTR{1}}.pvvvoxel) <= pthresh;
  case 3
    good = VS_PDATA{1}(VS_R{VS_RPTR{1}}.pvvvoxel) > pthresh;
  case 4
    good = VS_PDATA{1}(VS_R{VS_RPTR{1}}.pvvvoxel) >= pthresh;
  end
end

% deal with tisolated
good = good & ~(VS_TISOLATED(VS_R{VS_RPTR{1}}.pvv1) | VS_TISOLATED(VS_R{VS_RPTR{1}}.pvv2));

% pull out normals
pvv1ptsn = VS_TNORMALS(1:3,VS_R{VS_RPTR{1}}.pvv1(good));  % 3 x L
pvv2ptsn = VS_TNORMALS(1:3,VS_R{VS_RPTR{1}}.pvv2(good));  % 3 x L

% find which ones of those are pointed towards the camera
pos = campos(ax);
target = camtarget(ax);
v = pos-target;
pvv1ptsngood = dot(repmat(v',[1 size(pvv1ptsn,2)]),pvv1ptsn,1);  % 1 x L
pvv2ptsngood = dot(repmat(v',[1 size(pvv2ptsn,2)]),pvv2ptsn,1);  % 1 x L
good2 = pvv1ptsngood > 0 & pvv2ptsngood > 0;  % 1 x L

% for those, pull out coordinates
pvv1pts = VS_TXYZ(1:3,subscript(VS_R{VS_RPTR{1}}.pvv1(good),good2));  % 3 x M
pvv2pts = VS_TXYZ(1:3,subscript(VS_R{VS_RPTR{1}}.pvv2(good),good2));  % 3 x M

% move towards camera
pvv1pts = pvv1pts + movetocamera*(repmat(pos',[1 size(pvv1pts,2)]) - pvv1pts);
pvv2pts = pvv2pts + movetocamera*(repmat(pos',[1 size(pvv2pts,2)]) - pvv2pts);

% ok, draw new line objects.
prev = get(0,'CurrentFigure');  % a little hacky, oh well
set(0,'CurrentFigure',fig);
hold on;
lineobjs = plot3([pvv1pts(1,:); pvv2pts(1,:)], ...
                 [pvv1pts(2,:); pvv2pts(2,:)], ...
                 [pvv1pts(3,:); pvv2pts(3,:)]);
hold off;
set(0,'CurrentFigure',prev);

% set other attributes
set(lineobjs,'LineWidth',1, ...
             'Clipping','on');
