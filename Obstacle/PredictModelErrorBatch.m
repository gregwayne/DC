close all;
addpath ..;
MakeParameters;

if 1
    
    perror      = FFNet(3+2,100,1);
    gerror      = FFNet(3+2+3,100,1);
    oerror      = FFNet(3+2+env.O.mapN,200,1);
    cerror      = FFNet(env.O.mapN,100,1);  
    
end

batches = 10000;
csts    = zeros(batches,4);

fig     = figure(5);

addpath ../minFunc/;
options.Method      = 'lbfgs'; 
options.maxIter     = 30;
options.maxFunEvals = 1000;
options.display     = 'off'; 

nex = 1e4;

addpath ../Truck/;
addpath ..;
load 'lcontroller.mat';

MP          = zeros(3+2,nex);
VP          = zeros(1,nex);
MG          = zeros(3+2+3,nex);
VG          = zeros(1,nex);
MO          = zeros(3+2+env.O.mapN,nex);
VO          = zeros(1,nex);
MC          = zeros(env.O.mapN,nex);
VC          = zeros(1,nex);

for btch=1:batches
    
    % since these might also be training
    load 'pforward.mat';
    load 'gforward.mat';
    load 'oforward.mat';
    load 'ocdecoder.mat';
    
    t1=tic();
    if 1
        PredictModelError;
    end
    toc(t1);
    t2=tic();
    %load patterns_forward.mat; 
    
    ptheta          = perror.GetTheta();
    gtheta          = gerror.GetTheta();
    otheta          = oerror.GetTheta();
    ctheta          = cerror.GetTheta();
        
    [ptheta,cst]    = minFunc(@(par) FFCost(perror,par,{MP,VP}),ptheta,options);
    csts(btch,1)    = cst;
    [gtheta,cst]    = minFunc(@(par) FFCost(gerror,par,{MG,VG}),gtheta,options);
    csts(btch,2)    = cst;
    [otheta,cst]    = minFunc(@(par) FFCost(oerror,par,{MO,VO}),otheta,options);    
    csts(btch,3)    = cst;    
    [ctheta,cst]    = minFunc(@(par) FFCost(cerror,par,{MC,VC}),ctheta,options);    
    csts(btch,4)    = cst;

    perror.SetTheta(ptheta);
    save('perror.mat','perror');
    gerror.SetTheta(gtheta);
    save('gerror.mat','gerror');
    oerror.SetTheta(otheta);
    save('oerror.mat','oerror');
    cerror.SetTheta(ctheta);
    save('cerror.mat','cerror');
        
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