siml    = 10;

addpath '../Models/';
addpath '../Obstacle/';
addpath '../minFunc/';
addpath '..';

run ../Obstacle/MakeParameters;
[env,x]     = init_sim(env);

addpath '../Truck/';

G       = [];

addpath minFunc/;
options.Method      = 'lbfgs'; 
options.maxIter     = 1e5;
options.maxFunEvals = 1e5;
options.display     = 'on';
warning off;
    
LXF     = @(x,nil) LX(env.E,x);
LMF     = @(m,nil) LM(env.E,m);

LossXFn     = @(x,nil) LossX(env,x,[1,0]) + 10*LXF(x);
Forward     = @(x,u) SimDynamics(env,x,u,O.simCoarse);

us          = zeros(siml*env.O.periods,1);

[opt_us,cst] = minFunc(@(par) ControlCost(Forward,env.O.periods*siml,...
                                    x,par,LossXFn,LMF),us(:),options);
                                
us      = reshape(opt_us,size(us));
xs      = GenerateStates(Forward,x,us,siml*env.O.periods);

for t=1:size(xs,2)
    
    G       = SimAnimation(env, G, xs(:,t), us(t));   
    pause;    
    drawnow;

end