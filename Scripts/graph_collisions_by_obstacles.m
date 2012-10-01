% put data in collisions.mat

ntrials = size(collisions,1);
nrounds = size(collisions,2);

phats   = zeros(nrounds+1,1);
pcis    = zeros(nrounds+1,2);

xs      = 5:5:5*nrounds;

for i=1:nrounds
   
    [phats(i),pcis(i,:)] = binofit(sum(collisions(:,i)),ntrials);   
    
end

[serx, sery]    = DoubleSeries(xs,pcis(:,1),pcis(:,2));
sery            = min(sery,1);
sery            = max(sery,0);

pat = patch(serx,sery,[1 0 0],'BackFaceLighting','lit','EdgeAlpha',0);
axis([0 max(serx) 0 1]);
hold on;
line(xs,phats(:),'LineWidth',2);