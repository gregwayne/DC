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
           
    odists  = SenseObstacles(env2,y);    

    odists_pred = FProp(oforward,[p;psense0;odists0]);
    p_pred      = FProp(pforward,[p;psense0]);
    g_pred      = FProp(gforward,[p;psense0;gsense0]);
    c_pred      = FProp(ocdecoder,odists);
    
    MP(:,k) = [p;psense0];
    VP(:,k) = norm(SenseProprioceptive(y)-p_pred)^2;
    
    MG(:,k) = [p;psense0;gsense0];
    VG(:,k) = norm(SenseGoal(y)-g_pred)^2;    
         
    oc      = ObstacleCost(env2,y);
    MC(:,k) = odists;    
    VC(k)   = norm(oc-c_pred)^2;
      
    MO(:,k) = [p;psense0;odists0];
    VO(:,k) = norm(odists-odists_pred)^2;    
        
    
end

%save('patterns_forward.mat','MP','VP','MG','VG','MO','VO','MC','VC');