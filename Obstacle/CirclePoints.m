function [xs,ys] = CirclePoints(center,radius,n_edges)
    
    xs = radius*cos(2*pi*(1:n_edges)/n_edges)+center(1);
    ys = radius*sin(2*pi*(1:n_edges)/n_edges)+center(2);
        
end