function ix = InternalState(x)

    orig_dist   = norm(x(1:2));
    allo_angle  = atan2(x(2),x(1));
    theta_t     = x(3) - allo_angle;
    theta_c     = x(4) - allo_angle;    
    theta_c     = theta_c - theta_t;
    
    ix          = [orig_dist/100;...
                    sin(theta_t);cos(theta_t);...
                    sin(theta_c);cos(theta_c)];
        
end