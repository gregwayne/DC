function Q = EgoCentric(M)

    num         = size(M,2);
    Q           = zeros(5,num);
    
    for n=1:num
        
        x       = M(:,n);
        ix      = InternalState(x);
                    
        Q(:,n)  = ix;
        
    end

end