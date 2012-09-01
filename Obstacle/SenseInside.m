function inside = SenseInside(env,y)
    
    dists   = sqrt(sum(bsxfun(@minus,env.E.obstacles,y(1:2)).^2,1));
    mdist   = min(dists);
    if mdist < env.E.co % near obstacle
        inside = 1;
    else
        inside = 0;
    end

end