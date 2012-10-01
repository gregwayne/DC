function ex = SenseGoal(x)

    orig_dist   = norm(x(1:2));
    allo_angle  = atan2(x(2),x(1));
    theta_t     = x(3) - allo_angle;
   
    ex          = [orig_dist/100;sin(theta_t);cos(theta_t)];
    
end