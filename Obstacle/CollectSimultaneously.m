nex     = 5e5;

MakeParameters;
G = [];

us          = zeros(1,env.O.periods);
ys          = zeros(4,env.O.periods);

NS          = 1; % number of stages
siml        = 1; % length of "trial"

A           = zeros(5+env.O.mapN,nex*siml);
B           = zeros(3,nex*siml);
C           = zeros(3+5+env.O.mapN,nex*siml);
D           = zeros(5+env.O.mapN,nex*siml);

% addpath ../minFunc/;
% options.Method      = 'lbfgs'; 
% options.maxIter     = 100;
% options.maxFunEvals = 100;
% options.display     = 'off';
% warning off;

for k=1:nex
    
    if mod(k,100)==1
        disp(sprintf('Simultaneous Trial %d',k));
    end

    [env,y]         = init_sim(env);
        
    for ss=1:siml
        y0          = y;        
        ex          = SenseMap(env,y);
        ex_sim      = ex;

        ps          = zeros(3,NS);

        for state=1:NS
           
            %ps(:,state)  = hcontroller.FProp(SenseMap(env,y0));
            rangle       = 2*pi*rand;
            ps(:,state)  = [rand;sin(rangle);cos(rangle)];
            %ex_sim       = hcausal.FProp([ps(:,state);ex_sim]);

        end

        %[ps,cst]    = minFunc(@(par) HOCCost2(env,par,ex,hcausal),ps(:),options);
        
        rangle  = 2*pi*rand;
        p       = [rand;sin(rangle);cos(rangle)];
        %p       = ps(1:3) + 0.05*randn(3,1);
        
        x_fic       = ConvertBearingToVirtualPosition(E,p,y);

        % Generate Controls Here
        for t=1:env.O.periods 
            ix      = InternalState(x_fic);
            u       = lcontroller.FProp(ix);
            us(:,t) = u;
            ys(:,t) = y;      
            x_fic   = SimDynamics(env.E,x_fic,u,O.simCoarse);
            y       = SimDynamics(env.E,y,u,O.simCoarse);
        end
        
        ex2         = SenseMap(env,y);
        
        if 0
            map     = zeros(env.O.mapN,1);
            for t=1:env.O.periods
                [map,env.E.obstacles_detected] = ShootBeams(env,ys(:,t),map,1);
                
                G       = SimAnimation(env.E, G, ys(:,t), us(:,t));
                ta      = y0(3) - atan2(p(2),p(3));
                set(G.plan,'position',y0(1:2) - 100*p(1)*[cos(ta);sin(ta)]); 
                drawnow;
            end
        end
        
        A(:,(k-1)*siml + ss) = ex;
        B(:,(k-1)*siml + ss) = ps(1:3);
        C(:,(k-1)*siml + ss) = [ex;p];
        D(:,(k-1)*siml + ss) = ex2; % change this later to difference 
        
    end
    
end

patterns_hc  = {A,B,C,D};
save patterns_hc;