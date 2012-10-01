figure();
N               = 10;

load rs1.mat;
[pbs1,cbs1]     = BinSuccesses(rs{1},rs{2},N);
xs              = (1000/N)*(1:N);

[serx1,sery1]   = DoubleSeries(xs,cbs1(:,2),cbs1(:,1));
sery1           = min(sery1,1);
sery1           = max(sery1,0);

load rs2.mat;
[pbs2,cbs2]     = BinSuccesses(rs{1},rs{2},N);
[serx2,sery2]   = DoubleSeries(xs,cbs2(:,2),cbs2(:,1));
sery2           = min(sery2,1);
sery2           = max(sery2,0);

p1 = patch(serx1,sery1,[1 0 0],'BackFaceLighting','lit','EdgeAlpha',0);
alpha(p1,0.5);
p2 = patch(serx2,sery2,[0 1 1],'BackFaceLighting','lit','EdgeAlpha',0);
alpha(p2,0.5);
hold on;
line(xs,pbs1,'Color','k','LineWidth',4);
line(xs,pbs2,'Color','k','LineWidth',4);
xlim([100 1000]);
set(gca,'YTick',[0 0.5 1]);
set(gca,'XTick',linspace(100,1000,N));
%axis off;
