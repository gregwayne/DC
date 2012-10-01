function L = L(E,x)
    
    L       = norm(x(1:2)/100);
    ts      = x(3);
    tc      = x(4);
    
    ts      = mod(ts,2*pi);
    tc      = mod(tc,2*pi);
    ad      = mod(tc-ts,2*pi);

    acon    = E.acon;
    if ad > (acon) && ad < pi
        L   = L + 5*(ad-acon)^2;
    elseif ad > pi && ad < (2*pi-acon)
        L   = L + 5*((2*pi-acon)-ad)^2;
    end  
    
end