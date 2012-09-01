nex     = 1e3;
siml    = 30;

MakeParameters;

G           = [];

M           = zeros(3+2+env.O.mapN,nex*siml);
V           = zeros(3,nex*siml);
NS          = 8; % number of stages

addpath ../minFunc/;
options.Method      = 'lbfgs'; 
options.maxIter     = 1e4;
options.maxFunEvals = 1e4;
options.display     = 'off';
warning off;

us  = zeros(1,env.O.periods);
ys  = zeros(4,env.O.periods);
ps  = zeros(3,NS);

show_on = 1;

for k=1:nex
    
    %if mod(k,10)==1
        disp(sprintf('Controller Trial %d',k));
    %end  
    
    if 0
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
           
    for ss=1:siml

        y0          = y; 
        ps          = reshape(ps,[3,NS]);
        
        psense0     = SenseProprioceptive(y0);
        gsense0     = SenseGoal(y0);        
        osense0     = SenseObstacles(env,y0);
                
        psense      = psense0;
        gsense      = gsense0;
        osense      = osense0;

        for t=1:NS
           
            if ss==1
                ps(:,t) = FProp(hcontroller,[psense;gsense;osense]); 
            else
                if t < NS
                    ps(:,t) = ps(:,t+1);
                else
                    ps(:,t) = FProp(hcontroller,[psense;gsense;osense]);
                end
            end                
            
            psense     = FProp(pforward,[ps(:,t);psense]);            
            gsense     = FProp(gforward,[ps(:,t);psense;gsense]);
            osense     = FProp(oforward,[ps(:,t);psense;osense]);   
                        
        end
        
        yorig       = y;
        [ps,cst]    = minFunc(@(par) AdjointCost2(env,NS,3,par,...
                            psense0,gsense0,osense0,...
                            pforward,gforward,oforward,ocdecoder),...
                            ps(:),options);        
        CheckOpenLoop;
        y           = yorig;

        p           = ps(1:3);
                
        if siml > 1 || show_on
            y_fic       = ConvertBearingToVirtualPosition(E,p,y);

            % Generate Controls Here
            for t=1:env.O.periods 
                iy      = InternalState(y_fic);
                u       = FProp(lcontroller,iy);
                us(:,t) = u;
                ys(:,t) = y;      
                y_fic   = SimDynamics(env,y_fic,u,env.O.simCoarse);
                y       = SimDynamics(env,y,u,env.O.simCoarse);
            end            
        end
        
        if show_on
            
            subgoal     = ConvertBearingToAbsolutePosition(env.E,p,y0);
            for t=1:env.O.periods
                map                             = zeros(env.O.mapN,1);
                [map,env.E.obstacles_detected]  = ShootBeams(env,y,map,1); 
                G           = SimAnimation(env, G, ys(:,t), us(:,t));   
                set(G.plan,'position',subgoal);
                set(G.lineplan,'XData',yplan(1,:));
                set(G.lineplan,'YData',yplan(2,:));
                drawnow;
            end
        end
                      
        M(:,(k-1)*siml + ss) = [psense0;gsense0;osense0];
        V(:,(k-1)*siml + ss) = p;
        
    end
    
end

save('patterns_hc.mat','M','V');