function [cst,grd] = ControlCost(Forward,LT,x0,ms,LX,LM)

    xs      = zeros(length(x0),LT+1);
    xs(:,1) = x0;

    ms      = reshape(ms,[length(ms)/LT,LT]);
        
    gradxs  = zeros(size(xs));
    gradms  = zeros(size(ms));
               
    cst = 0;
    for t=1:LT
       
        xs(:,t+1)   = Forward(xs(:,t),ms(:,t));
        cst         = cst + LX(xs(:,t+1)) + LM(ms(:,t));
        
        gradxs(:,t+1)   = NumericalGradient(LX,xs(:,t+1),[]);
        gradms(:,t)     = NumericalGradient(LM,ms(:,t),[]);
                
    end

    for t=(LT+1):-1:2
        
        [Jx,Jm]         = NumericalJacobian(Forward,xs(:,t-1),ms(:,t-1));
        
        gradxs(:,t-1)   = gradxs(:,t-1) + Jx'*gradxs(:,t);
        gradms(:,t-1)   = gradms(:,t-1) + Jm'*gradxs(:,t);
                
    end
    
    cst     = cst/LT;
    grd     = gradms(:)/LT;
    
end