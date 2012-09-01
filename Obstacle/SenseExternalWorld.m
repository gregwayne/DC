function ex = SenseExternalWorld(E,x)

    orig_dist   = norm(x(1:2));
    allo_angle  = atan2(x(2),x(1));
    theta_t     = x(3) - allo_angle;
    theta_c     = x(4) - allo_angle;    
    theta_c     = theta_c - theta_t;
    
    min_dist = inf;
    min_displace = zeros(2,1);
    min_idx  = 1;
    for i=1:E.nobstacles
        dist = norm(x(1:2)-E.obstacles(:,i));
        if dist < min_dist
            min_dist    = dist;
            min_idx     = i;
            min_displace = x(1:2) - E.obstacles(:,i);
        end        
    end        
    
    o_angle     = atan2(min_displace(2),min_displace(1));
    theta_o     = x(3) - o_angle;
    
    ex          = [orig_dist/100;...
                    sin(theta_t);cos(theta_t);...
                    min_dist/100;...
                    sin(theta_o);cos(theta_o);...
                    sin(theta_c);cos(theta_c)];
        
end