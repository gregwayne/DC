function [env2,y] = init_sim(env,varargin)
    
    % Make persistent by uncommenting
    % if you want repeated initial conditions
    if nargin > 1
        d   = varargin{1};
    else
        d   = 500;
    end
    
    %persistent x trailer;

    env2 = struct;
    env2 = env;
    %if isempty(trailer)
        theta   = 2*pi*rand;
        px      = cos(theta);
        py      = sin(theta);
        rnd_dst = rand*d;
        %if isempty(trailer)
        if isempty(env.E.obstacles)
            env2.E.obstacles = 600*(rand(2,env.E.nobstacles)-0.5);
        end
        %end
        trailer = [rnd_dst*px;rnd_dst*py;2*pi*rand];
        
        x       = [trailer;trailer(3)+(rand-0.5)*pi/2];
    %end
    
    y = x;
       
end
