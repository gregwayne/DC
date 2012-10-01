function cst = ObstacleCost(env,x)

    dxs     = bsxfun(@minus,env.E.obstacles,x(1:2));
    dists   = sum(dxs.^2,1);
    mdist   = min(dists);
   
    cst     = sum(exp(-mdist/(2*env.E.co^2))) + 2*sum(exp(-mdist/(2*env.E.disk^2)));
        
end