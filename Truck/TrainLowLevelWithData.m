clear all;
close all;
load patterns_c;

addpath ..;
addpath ../minFunc/;

options.Method          = 'lbfgs'; 
options.maxIter         = 30;
options.maxFunEvals     = 1000;
options.display         = 'on'; 

M = patterns_c{1};
Q = patterns_c{2};
V = patterns_c{3};

N0=5;
N1=30;
N2=1;

lcontroller = FFNet(N0,N1,N2);

theta       = lcontroller.GetTheta();

for btch=1:10
    P           = randperm(size(Q,2));
    idxs        = P(1:1e5);
    [theta,cst] = minFunc(@(par) FFCost(lcontroller,par,{Q(:,idxs),V(:,idxs)}),theta,options);        
end

lcontroller.SetTheta(theta);
save('lcontroller.mat','lcontroller');