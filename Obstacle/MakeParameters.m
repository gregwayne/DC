E               = struct;
E.cw            = 6;
E.cl            = 6;
E.co            = 40;
E.disk          = 20; % was 25
E.tw            = 6;
E.tl            = 14;
E.r             = 0.2;
E.nu            = 1;

E.acon          = pi/2-pi/6;
E.ucon          = 0.95;

O               = struct;
O.mapN          = 30; 
O.max_trial     = 100;
O.max_steps     = 6000;
O.plot_every    = 6; % was 4
O.simCoarse     = 6;  
O.periods       = 15;
O.rel_cost      = 10;

E.nobstacles    = 40; % was 20
E.jiggle_rate   = 1;
E.obstacles_detected = [];

env             = struct;
env.E           = E;
env.O           = O;

%save('env.mat','env');