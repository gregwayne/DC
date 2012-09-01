function Q = SenseTransform(E,M)

    num         = size(M,2);
    Q           = zeros(8,num);
    
    for n=1:num
        
        x       = M(:,n);
        ex      = SenseExternalWorld(E,x);
                    
        Q(:,n)  = ex;
        
    end

end