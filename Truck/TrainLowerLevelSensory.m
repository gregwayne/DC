close all;
addpath ../Models/;
addpath ..;
MakeParameters;
rng('shuffle');

if 0
    lforward    = FFNet(6,50,5);
end

batches = 1e3;
csts    = zeros(batches,4);

fig     = figure(5);

addpath ../minFunc/;
options.Method      = 'lbfgs'; 
options.maxIter     = 20;
options.maxFunEvals = 1000;
options.display     = 'off'; 

nex = 1e5;

M          = zeros(6,nex);
V          = zeros(5,nex);

for btch=1:batches
    
    t1=tic();
    if 1
       TrainLLSensory;
    end
    toc(t1);
    
    t2=tic();
    
    lftheta         = lforward.GetTheta();  

    [lftheta,cst]    = minFunc(@(par) FFCost(lforward,par,{MP,VP}),lftheta,options);
    csts(btch,1)    = cst;

    lforward.SetTheta(lftheta);
    save('lforward.mat','lforward');
        
    figure(5);
    set(0,'CurrentFigure',fig);
    xa=1:btch;
    loglog(xa,csts(xa,1),'r');

end