function [nearness,angles] = SenseObstacles(env,x)

    % Under Development

    [map,obstacle_list,angles]    = DepthMap(env,x);
        
    nearness        = 2./(1+0.1*map);
    nearness        = tanh(nearness);    
    
end