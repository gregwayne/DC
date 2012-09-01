E               = struct;
E.cw            = 6;
E.cl            = 6;
E.co            = 40;
E.disk          = 25;
E.tw            = 6;
E.tl            = 14;
E.r             = 0.2;
E.nu            = 1;

E.nobstacles    = 20;
E.jiggle_rate   = 4;
E.obstacles_detected = [];

O               = struct;
O.mapN          = 30; 
O.max_trial     = 100;
O.max_steps     = 2000;
O.plot_every    = 4;
O.simCoarse     = 6;  
O.periods       = 15;
O.rel_cost      = 10;

env             = struct;
env.E           = E;
env.O           = O;

%save('env.mat','env');