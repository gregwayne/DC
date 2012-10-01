function [Jx,Ju] = NumericalGradient(L,x0,u0)

    eps     = 1e-3;
    teps    = 2*eps;
    
    nx = numel(x0);
    nu = numel(u0);
    
    Jx  = zeros(1,nx);
    Ju  = zeros(1,nu);
            
    parfor i=1:nx
        
        xu   = x0;
        xl   = x0;
        
        xu(i) = xu(i) + eps;
        xl(i) = xl(i) - eps;
                
        Jx(:,i)   = (L(xu,u0) - L(xl,u0))/(teps);
        
    end
        
    parfor i=1:nu
        
        uu   = u0;
        ul   = u0;
        
        uu(i) = uu(i) + eps;
        ul(i) = ul(i) - eps;
        
        Ju(:,i)   = (L(x0,uu) - L(x0,ul))/(teps);
        
    end

end