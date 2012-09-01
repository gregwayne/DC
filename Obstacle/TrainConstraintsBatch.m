MakeParameters;
addpath ..;

if 0
    
    pcon    = FFNet(2,100,1);
    gcon    = FFNet(3,100,1);
    ocon    = FFNet(env.O.mapN,100,1);

else
   
    load 'pcon.mat';
    load 'gcon.mat';
    load 'ocon.mat';
    
end

batches = 1e4;

csts    = zeros(batches,3);

fig     = figure(5);

addpath ../minFunc/;
options.Method      = 'lbfgs'; 
options.maxIter     = 20;
options.maxFunEvals = 1000;
options.display     = 'off'; 

for btch=1:batches
    
    TrainConstraints;
    load 'constraints.mat';

    Q               = randperm(size(MP,2)); 
    
    FFCost(pcon,pcon.GetTheta(),{MP(:,Q),VP(:,Q)})
    
    ptheta          = pcon.GetTheta();
    [ptheta,cst]    = minFunc(@(par) FFCost(pcon,par,{MP(:,Q),VP(:,Q)}),ptheta,options);
    pcon.SetTheta(ptheta);
    save('pcon.mat','pcon');
    csts(btch,1)    = cst;
    
    FFCost(gcon,gcon.GetTheta(),{MG(:,Q),VG(:,Q)})

    gtheta          = gcon.GetTheta();
    [gtheta,cst]    = minFunc(@(par) FFCost(gcon,par,{MG(:,Q),VG(:,Q)}),gtheta,options);
    gcon.SetTheta(gtheta);
    save('gcon.mat','gcon');    
    csts(btch,2)    = cst;
    
    FFCost(ocon,ocon.GetTheta(),{MO(:,Q),VO(:,Q)})

    otheta          = ocon.GetTheta();
    [otheta,cst]    = minFunc(@(par) FFCost(ocon,par,{MO(:,Q),VO(:,Q)}),otheta,options);
    ocon.SetTheta(otheta); 
    save('ocon.mat','ocon');    
    csts(btch,3)    = cst;

    set(0,'CurrentFigure',fig);
    subplot(2,1,1);
    xa=1:btch;
    loglog(xa,csts(xa,1),'r',xa,csts(xa,2),'g',xa,csts(xa,3),'b');
    subplot(2,1,2);
    plot(xa,csts(xa,1),'r',xa,csts(xa,2),'g',xa,csts(xa,3),'b');    
    drawnow;
   
end