MakeParameters;

addpath ../Truck/;
addpath ..;
load 'lcontroller.mat';
load 'pforward.mat';
load 'gforward.mat';
load 'oforward.mat';
G = [];

env.E.nobstacles = 1;
p           = zeros(3,1);

while 1
            
    [env,y]     = init_sim(env);
    y0          = y;

    p(1)        = rand;   
    rangle      = rand*2*pi;
    p(2)        = sin(rangle);
    p(3)        = cos(rangle);
    
    y_fic       = ConvertBearingToVirtualPosition(env.E,p,y);
    subgoal     = ConvertBearingToAbsolutePosition(env.E,p,y);
    
    psense0 = SenseProprioceptive(y0);
    gsense0 = SenseGoal(y0);        
    osense0 = SenseObstacles(env,y0);
                   
    dgsense  = FProp(gforward,[p;psense0;gsense0]);
    dpsense  = FProp(pforward,[p;psense0]);
    dosense  = FProp(oforward,[p;psense0;osense0]);         
    
    % Generate Controls Here
    for t=1:env.O.periods 
        iy      = InternalState(y_fic);
        u       = FProp(lcontroller,iy);
        y_fic   = SimDynamics(env.E,y_fic,u,env.O.simCoarse);
        y       = SimDynamics(env.E,y,u,env.O.simCoarse);
        
        %map                             = zeros(env.O.mapN,1);
        %[map,env.E.obstacles_detected]  = ShootBeams(env,y,map,1); 
        
        G       = SimAnimation(env.E,G,y,u);
        set(G.plan,'position',subgoal(1:2));
        drawnow;        
    end
    
    psensef = SenseProprioceptive(y);
    gsensef = SenseGoal(y);        
    osensef = SenseObstacles(env,y);
                        
    figure(4);
    disp(norm([dpsense+psense0-psensef;dgsense+gsense0-gsensef;dosense+osense0-osensef])^2);
    bar([dpsense+psense0-psensef;dgsense+gsense0-gsensef;dosense+osense0-osensef]);
    figure(5);
    bar([psensef;gsensef;osensef]);
    figure(1);
    drawnow;
    pause(1);
    
    
end
