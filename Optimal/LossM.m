function L = LossM(env,m,flg)
    
    L       = 0;
    
    mot1lb  = 0.1;
    mot1ub  = 0.9;
    if m(1) < mot1lb
        L = L + 100*(m(1) - mot1lb)^2;
    elseif m(1) > mot1ub
        L = L + 100*(m(1) - mot1ub)^2;
    end
    
    L   = L + 100*(norm(m(2:3))^2 - 1)^2;
        
end