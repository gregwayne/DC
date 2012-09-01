for step=1:NS

    p           = ps((3*(step-1)+1):3*step);   
    y0          = y;
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

    subgoal     = ConvertBearingToAbsolutePosition(env.E,p,y0);
    for t=1:env.O.periods
        map                             = zeros(env.O.mapN,1);
        [map,env.E.obstacles_detected]  = ShootBeams(env,y,map,1); 
        G           = SimAnimation(env, G, ys(:,t), us(:,t));   
        set(G.plan,'position',subgoal);  
        drawnow;                
    end
end