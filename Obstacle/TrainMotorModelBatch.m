close all;
addpath ..;
MakeParameters;
rng('shuffle');

if 0
    hcontroller = FFNet(3+2+env.O.mapN,100,3);
else
    load '../Models/hcontroller.mat';
end
    
batches = 1e4;
csts    = zeros(batches,1);

fig     = figure(5);

Q = [];
R = [];

for btch=1:batches
    
    % these may be learning simultaneously
    load '../Models/pforward.mat';
    load '../Models/gforward.mat';
    load '../Models/oforward.mat';
    load '../Models/ocdecoder.mat';
    load '../Models/lcontroller.mat';
    
    if 1
        %TrainMotorModel;
        TMMParallel;
    end
    
    Q   = [Q,MF];
    R   = [R,VF];
    addpath ../minFunc/;
    options.Method      = 'lbfgs'; 
    options.maxIter     = 20;
    options.maxFunEvals = 1000;
    options.display     = 'on';
    
    htheta          = hcontroller.GetTheta();
    [htheta,cst]    = minFunc(@(par) FFCost(hcontroller,par,{Q,R}),htheta,options);

    hcontroller.SetTheta(htheta);
    save('hcontroller.mat','hcontroller');

    csts(btch,1)    = cst;
    
    figure(5);
    set(0,'CurrentFigure',fig);
    plot(1:btch,csts(1:btch,1),'r');
    drawnow;

end