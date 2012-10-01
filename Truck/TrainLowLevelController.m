close all;
nex = 10;

addpath '../';
addpath '../minFunc/';
addpath '../Optimal/';
MakeParameters;

Forward         = @(x,u) TruckDynamics(env,x,u,O.simCoarse);
options.Method  = 'lbfgs'; 
options.maxIter = 1000;	
options.display = 'off'; 
us              = zeros(env.E.nu,env.O.periods);

G = [];

patterns = {};
M = zeros(4,nex*env.O.periods);
Q = zeros(5,nex*env.O.periods);
V = zeros(1,nex*env.O.periods);

tic
for k=1:nex
    if mod(k,10)==1
        disp(sprintf('%d of %d',k,nex));
    end
    us          = 0*us;
    
    dx_r        = rand*100;
    dx_theta    = 2*pi*rand;
    
    dx          = dx_r*[cos(dx_theta);sin(dx_theta)];
    
    trailer = [dx(1:2);2*pi*rand];
    x       = [trailer;(rand-0.5)*(pi-pi/32)+trailer(3)];
            
    LXF     = @(x,nil) LX(E,x);
    LMF     = @(m,nil) LM(E,m);

    % Generate Control Here
    [opt_us,cst] = minFunc(@(par) ControlCost(Forward,env.O.periods,...
                                    x,par,LXF,LMF),us(:),options);
                
    us      = reshape(opt_us,size(us));
    xs      = GenerateStates(Forward,x,us,env.O.periods);
    
    op                      = env.O.periods;
    M(:,(k-1)*op+1:k*op)    = xs;
    V(:,(k-1)*op+1:k*op)    = us;
        
%     u       = us(:,1);    
%     G       = SimAnimation(env, G, x, u);
%     if k==1
%         G.plan = line([0;0],[0;0],'Color','black');
%     end
%     set(G.plan,'Xdata',xs(1,:),'Ydata',xs(2,:));
%     drawnow;
    
end

Q               = EgoCentric(M);
patterns_c      = {M,Q,V};
save patterns_c;