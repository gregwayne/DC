function [map,obstacle_list,angles] = DepthMap(env,x)

    map                             = zeros(env.O.mapN,1);
    [map,obstacle_list,angles]      = ShootBeams(env,x,map,0);
                
end