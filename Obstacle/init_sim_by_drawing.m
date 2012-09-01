function [env] = init_sim_by_drawing(env)
        
    theta   = 2*pi*rand;
    px      = cos(theta);
    py      = sin(theta);
        
    c=1;
    while 1
        [cx,cy] = ginput(1);
        if isempty(cx)
            break;
        end
        env.E.obstacles(:,c) = [cx;cy];
        c = c+1;
    end
    
    env.E.obstacles     = env.E.obstacles(:,1:(c-1));
    env.E.nobstacles    = c-1; 
        
end
