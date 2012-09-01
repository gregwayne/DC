function y      = HForward(env,y,p,lcontroller)

    y_fic       = ConvertBearingToVirtualPosition(env.E,p,y);

    for t=1:env.O.periods
        iy      = InternalState(y_fic);
        u       = FProp(lcontroller,iy);    
        y_fic   = SimDynamics(env,y_fic,u,env.O.simCoarse);
        y       = SimDynamics(env,y,u,env.O.simCoarse);
    end 
            
end