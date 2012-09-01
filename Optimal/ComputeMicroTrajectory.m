function [xs,us]    = ComputeMicroTrajectory(env,x0,ms,lcontroller)

    LT  = size(ms,2);
    xs  = zeros(length(x0),LT*env.O.periods);

    x   = x0;
    
    for step=1:LT

        x_fic       = ConvertBearingToVirtualPosition(env.E,ms(:,step),x);

        for t=1:env.O.periods
                        
            ix      = InternalState(x_fic);
            u       = FProp(lcontroller,ix);    
            
            xs(:,(step-1)*env.O.periods+t)  = x;
            us(:,(step-1)*env.O.periods+t)  = u;            
            
            x_fic   = SimDynamics(env,x_fic,u,env.O.simCoarse);
            x       = SimDynamics(env,x,u,env.O.simCoarse);

        end                
        
    end

end