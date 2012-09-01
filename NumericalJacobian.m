function [Jx,Ju] = NumericalJacobian(Forward,x0,u0)

    eps     = 1e-3;
    teps    = 2*eps;
    
    nx = length(x0);
    nu = length(u0);
    
    esx = eye(nx);
    esu = eye(nu);
    
    Jx  = zeros(nx,nx);
    Ju  = zeros(nx,nu);
    
    for i=1:nx
        
        xu   = x0 + eps*esx(:,i);
        xl   = x0 - eps*esx(:,i);
        
        Jx(:,i)   = (Forward(xu,u0) - Forward(xl,u0))/(teps);
        
    end
    
    for i=1:nu
        
        uu   = u0 + eps*esu(:,i);
        ul   = u0 - eps*esu(:,i);
        
        Ju(:,i)   = (Forward(x0,uu) - Forward(x0,ul))/(teps);
        
    end

end