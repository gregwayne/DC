function [env,y] = init_sim(env)
    
    % If you want repeated initial conditions,
    % make persistent by uncommenting.
    
    %persistent x trailer;
    
    %if isempty(trailer)
        trailer = [150*(rand-0.5);150*(rand-0.5);2*pi*rand];
        x       = [trailer;trailer(3)+(rand-0.5)*pi/2];
    %end
    
    y = x;
       
end
