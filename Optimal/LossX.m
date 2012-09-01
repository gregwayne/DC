function L = LossX(env,x,flg)

    E       = env.E;
    
    L1      = norm(x(1:2))/100;
    L2      = 0;
    
    % Obstacles
    
    obsP    = flg;
    
    if obsP
        dxs     = bsxfun(@minus,env.E.obstacles,x(1:2));
        dists   = sum(dxs.^2,1);
        mdist   = min(dists);

        L2       = exp(-mdist/(2*env.E.co^2)) + 2*exp(-mdist/(2*env.E.disk^2));
    end    
    
    L   = L1 + env.O.rel_cost*L2;
    
end