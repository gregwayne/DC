function xs = GenerateStates(Forward,x0,us,P)

    xs      = zeros(length(x0),P);
    xs(:,1) = x0;
    
    for t=1:P-1
       
        
        xs(:,t+1) = Forward(xs(:,t),us(t));
        
    end

end