close all;
addpath('../');
MakeParameters;
O = env.O;
env.E.nobstacles = 1;

load 'hcausal.mat';
addpath('../Truck');
load 'lcontroller.mat';        

for fignum=1:100
    
figstr = int2str(fignum);    

G = [];

env.E.obstacles = 150*(rand(2,env.E.nobstacles)-0.5);
x               = 150*(rand(2,1)-0.5);

trailer     = [x(1:2);2*pi*rand];
y           = [trailer;0.2*(rand-0.5)*(pi-pi/16)+trailer(3)];

G           = SimAnimation(env.E, G, y, 0);
dp          = 100;
axis([-dp,dp,-dp,dp]);
%[ox,oy]     = ginput(1); % place obstacle
%env.E.obstacles = [ox;oy];
G           = SimAnimation(env.E, G, y, 0);
axis([-dp,dp,-dp,dp]);
exb         = SenseExternalWorld(env.E,y);

delta       = 10;

xs          = -dp:delta:dp;
ys          = -dp:delta:dp;

xN          = length(xs);
yN          = length(ys);

C1          = zeros(xN,yN);
C2          = zeros(xN,yN);

FPT         = cell(xN,yN);

LossFn      = @(ex,p) LossEx(ex,env.E.co) + LossP(p);

for i=1:length(xs)
    
    for j=1:length(ys)
        
        posx        = xs(i);
        posy        = ys(j);        
        p           = ConvertPositionToBearing(E,y,[posx;posy]);

        exm         = hcausal.FProp([p(:);exb]);
        ta          = y(3) - atan2(p(2),p(3));
        set(G.plan,'position',y(1:2) - 100*p(1)*[cos(ta);sin(ta)]);     
        drawnow;

        x_fic       = ConvertBearingToVirtualPosition(E,p,y);
        y0          = y;
        % Generate Controls Here
        for t=1:env.O.periods 
            ix      = InternalState(x_fic);
            u       = lcontroller.FProp(ix);    
            x_fic   = SimDynamics(env.E,x_fic,u,O.simCoarse);
            y0      = SimDynamics(env.E,y0,u,O.simCoarse);
        end        

        FPT{i,j}    = y0(1:2);
        
        exf         = SenseExternalWorld(env.E,y0);
        C1(i,j)     = LossFn(exm,p);
        C2(i,j)     = LossFn(exf,p);
        
    end
    
end

delete(G.plan);
axis([-dp dp -dp dp]);

minC = min(min(C1(:)),min(C2(:)));
maxC = max(max(C1(:)),max(C2(:)));

spanC = [minC maxC];

% figure(2);
% subplot(2,1,1);
% imagesc(xs,ys,C1,spanC);
% set(gca,'YDir','normal');
% hold on;
[i1,j1] = find(C1 == min(C1(:)));
% plot(xs(j1),ys(i1),'Marker','.','MarkerSize',20,'Color',[1 0 0]);
% axis square;
% colorbar;
% subplot(2,1,2);
% imagesc(xs,ys,C2,spanC);
% set(gca,'YDir','normal');
% hold on;
[i2,j2] = find(C2 == min(C2(:)));
% plot(xs(j2),ys(i2),'Marker','.','MarkerSize',20,'Color',[0 1 0]);
% axis square;
% colorbar;
% saveas(gcf,strcat('Data/pred_cost_',figstr),'eps2c');
% saveas(gcf,strcat('Data/pred_cost_',figstr),'fig');
% figure(1);
% text(xs(j1),ys(i1),'.','FontSize',100,'Color',[1 0 0]);
% text(xs(j2),ys(i2),'.','FontSize',100,'Color',[0 1 0]);
% saveas(gcf,strcat('Data/cost_arena_',figstr),'eps2c');
% saveas(gcf,strcat('Data/cost_arena_',figstr),'fig');
% close all;
cost_diff(fignum) = abs(C2(i2,j2)-C2(i1,j1));
cost_max(fignum)  = max(C2(:));
cost_min(fignum)  = min(C2(:));
end

figure(3);
hist(cost_diff./(cost_max-cost_min),50);
fixfig(3);



