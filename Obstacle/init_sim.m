function [env2,y] = init_sim(env)
    
    % Make persistent by uncommenting
    % if you want repeated initial conditions
    
    %persistent x trailer;
    env2 = struct;
    env2 = env;
    %if isempty(trailer)
        theta   = 2*pi*rand;
        px      = cos(theta);
        py      = sin(theta);
        rnd_dst = rand*1000;
        %if isempty(trailer)
            env2.E.obstacles = 600*(rand(2,env.E.nobstacles)-0.5);
        %end
        trailer = [rnd_dst*px;rnd_dst*py;2*pi*rand];
        
        x       = [trailer;trailer(3)+(rand-0.5)*pi/2];
    %end
 
    y = x;
       
end
