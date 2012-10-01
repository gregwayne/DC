close all;
addpath ../Models/;
MakeParameters;
rng('shuffle');

if 0
    pforward    = FFNet(3+2,200,2);
    gforward    = FFNet(3+2+3,300,3);
    oforward    = FFNet(3+2+env.O.mapN,400,env.O.mapN);
    ocdecoder   = FFNet(env.O.mapN,200,1);    
else
    load '../Models/pforward.mat';
    load '../Models/gforward.mat';
    load '../Models/oforward.mat';
    load '../Models/ocdecoder.mat';
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

load '../Models/lcontroller.mat';

MP          = zeros(3+2,nex);
VP          = zeros(2,nex);
MG          = zeros(3+2+3,nex);
VG          = zeros(3,nex);
MO          = zeros(3+2+env.O.mapN,nex);
VO          = zeros(env.O.mapN,nex);
MC          = zeros(env.O.mapN,nex);
VC          = zeros(1,nex);

MPR         = [];
VPR         = [];
MGR         = [];
VGR         = [];
MOR         = [];
VOR         = [];
MCR         = [];
VCR         = [];

for btch=1:batches
    
    t1=tic();
    if 1
       TrainSensoryForwardModel;
    end
    toc(t1);
    
    t2=tic();
    
    ptheta          = pforward.GetTheta();
    gtheta          = gforward.GetTheta();
    otheta          = oforward.GetTheta();
    octheta         = ocdecoder.GetTheta();    

    [ptheta,cst]    = minFunc(@(par) FFCost(pforward,par,{MP,VP}),ptheta,options);
    csts(btch,1)    = cst;
    [gtheta,cst]    = minFunc(@(par) FFCost(gforward,par,{MG,VG}),gtheta,options);
    csts(btch,2)    = cst;
    [otheta,cst]    = minFunc(@(par) FFCost(oforward,par,{MO,VO}),otheta,options);    
    csts(btch,3)    = cst;    
    [octheta,cst]   = minFunc(@(par) FFCost(ocdecoder,par,{MC,VC}),octheta,options);    
    csts(btch,4)    = cst;

    pforward.SetTheta(ptheta);
    save('pforward.mat','pforward');
    gforward.SetTheta(gtheta);
    save('gforward.mat','gforward');
    oforward.SetTheta(otheta);
    save('oforward.mat','oforward');
    ocdecoder.SetTheta(octheta);
    save('ocdecoder.mat','ocdecoder');
        
    figure(5);
    set(0,'CurrentFigure',fig);
    subplot(2,1,1);
    xa=1:btch;
    loglog(xa,csts(xa,1),'r',xa,csts(xa,2),'g',xa,csts(xa,3),'b',xa,csts(xa,4),'m');
    subplot(2,1,2);
    plot(xa,csts(xa,1),'r',xa,csts(xa,2),'g',xa,csts(xa,3),'b',xa,csts(xa,4),'m');    
    drawnow;
    toc(t2);

end