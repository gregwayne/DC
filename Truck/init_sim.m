function [env,y] = init_sim(env)
    
    % If you want repeated initial conditions,
    % make persistent by uncommenting.
    
    %persistent x trailer;
    
    %if isempty(trailer)
        theta   = 2*pi*rand;
        d       = 500*rand;
        trailer = [d*sin(theta);d*cos(theta);2*pi*rand];
        %trailer = [150*(rand-0.5);150*(rand-0.5);2*pi*rand];
        x       = [trailer;trailer(3)+(rand-0.5)*pi/2];
    %end
    
    y = x;
       
end
