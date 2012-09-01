function closure = TimeCounter(n)

    tick   = 0;
    function tock = counter(reset)
        
        tick    = tick + 1;
        tock    = mod(tick,n)==0;
        if tock
            tick = 0;
        end
        
        if reset
            tick = 0;
        end
        
    end

    closure = @(reset) counter(reset);

end