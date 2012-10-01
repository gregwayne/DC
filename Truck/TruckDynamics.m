function x = TruckDynamics(env,x,u,ti)
    
    E       = env.E;
    
    r       = E.r;
    dc      = E.cl;
    ds      = E.tl;
    
    for tt=1:ti

        ts      = x(3);
        tc      = x(4);

        A       = r*cos(u);
        B       = A*cos(tc-ts);
        C       = A*sin(tc-ts);

        cts     = cos(ts);
        sts     = sin(ts);

        x(1)    = x(1) - B*cts;
        x(2)    = x(2) - B*sts;

        ctc     = cos(tc);
        stc     = sin(tc);

        su      = sin(u);

        ts      = atan2(ds*sts-C*cts,ds*cts+C*sts);
        tc      = atan2(dc*stc-r*ctc*su,dc*ctc+r*stc*su);
        
        ts      = mod(ts,2*pi);
        tc      = mod(tc,2*pi);
                
        x(3)    = ts;
        x(4)    = tc;
        
    end
    
end