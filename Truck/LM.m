function L = L(E,m)

    L       = 0;
 
    ucon    = E.ucon;
    if abs(m) > ucon
        L   = L + 2*(abs(m)-ucon)^2;
    end   

end