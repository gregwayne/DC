parfor k=1:nex  

    [env2,y]    = init_sim(env);
    y0          = y;
     
    p           = zeros(3,1);
    p(1)        = rand;   
    rangle      = rand*2*pi;
    p(2)        = sin(rangle);
    p(3)        = cos(rangle);
    
    psense0     = SenseProprioceptive(y0);
    gsense0     = SenseGoal(y0);        
    odists0     = SenseObstacles(env2,y0);
        
    y_fic       = ConvertBearingToVirtualPosition(env2.E,p,y);
    subgoal     = ConvertBearingToAbsolutePosition(env2.E,p,y);
              
    % Generate Controls Here
    for t=1:env2.O.periods 
        iy      = InternalState(y_fic);
        u       = FProp(lcontroller,iy); 
        y       = SimDynamics(env2,y,u,env2.O.simCoarse);
        y_fic   = SimDynamics(env2,y_fic,u,env2.O.simCoarse);                
    end            
           
    M(:,k)      = [p;psense0];
    VP(:,k)     = SenseProprioceptive(y);
                
end