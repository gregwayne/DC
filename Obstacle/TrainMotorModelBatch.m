close all;
addpath ..;
addpath ../Truck;
MakeParameters;
rng('shuffle');

if 1
    load 'pforward.mat';
    load 'gforward.mat';
    load 'oforward.mat';
    load 'ocdecoder.mat';
    load 'lcontroller.mat';
end

if 0
    hcontroller = FFNet(3+2+env.O.mapN,200,3);
else
    load 'hcontroller.mat';
end
    
batches = 1e4;
csts    = zeros(batches,1);

fig     = figure(5);

Q = [];
R = [];

for btch=1:batches
    
    % these may be learning simultaneously
    load 'pforward.mat';
    load 'gforward.mat';
    load 'oforward.mat';
    load 'ocdecoder.mat';
    load 'lcontroller.mat';
    
    if 1
        TrainMotorModel;
        %TMMCloud;
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