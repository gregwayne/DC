siml    = 15;

addpath '../Models/';
addpath '../Obstacle/';
addpath '../minFunc/';
addpath '..';
load_networks;
MakeParameters;

G       = [];

addpath minFunc/;
options.Method      = 'lbfgs'; 
options.maxIter     = 1000;
options.maxFunEvals = 1000;
options.display     = 'off';
warning off;
    
if 1
    
    theta   = rand;
    px      = cos(theta);
    py      = sin(theta);
    trailer = 500*[(rand-0.5);(rand-0.5);2*pi*rand];
    y       = [trailer;trailer(3)+(rand-0.5)*pi/2];

    env.E.obstacles = zeros(2,100);
    G           = SimAnimation(env, G, y, 0);   
    env         = init_sim_by_drawing(env);
    
else
    
    [env,y]     = init_sim(env);
    
end

ps          = 0.05*randn(3,siml);
Forward     = @(y,m) HForward(env,y,m,lcontroller);
flgs        = [0,0;1,0];
%flgs        = [1,0];
%for attempt=1:100
tic
[ps,cst]    = ComputeOpenLoop(Forward,siml,y,ps,...
                            @(x,p) LossX(env,x,p),...
                            @(m,p) LossM(env,m,p),...
                                options,flgs);
toc

ys(:,1)     = y;
ps          = reshape(ps,[3,siml]);

[ys,us]     = ComputeMicroTrajectory(env,y,ps,lcontroller);
for t=1:size(ys,2)

    G       = SimAnimation(env, G, ys(:,t), 0);   
    if t==1
        set(G.lineplan,'XData',ys(1,:));
        set(G.lineplan,'YData',ys(2,:));
    end
    pause;    
    drawnow;

end
%end