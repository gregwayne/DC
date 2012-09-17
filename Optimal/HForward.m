function x      = HForward(env,x,m,lcontroller)

    x_fic       = ConvertBearingToVirtualPosition(env.E,m,x);

    for t=1:env.O.periods
        ix      = InternalState(x_fic);
        u       = FProp(lcontroller,ix);    
        x_fic   = SimDynamics(env,x_fic,u,env.O.simCoarse);
        x       = SimDynamics(env,x,u,env.O.simCoarse);
    end 
            
end