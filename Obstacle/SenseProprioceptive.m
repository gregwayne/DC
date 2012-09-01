function ex = SenseProprioceptive(x)

    allo_angle  = atan2(x(2),x(1));
    theta_t     = x(3) - allo_angle;
    theta_c     = x(4) - allo_angle;    
    theta_c     = theta_c - theta_t;
        
    ex          = [sin(theta_c);cos(theta_c)];
        
end