E               = struct;
E.cw            = 6;
E.cl            = 6;
E.co            = 6;
E.tw            = 6;
E.tl            = 14;
E.r             = 0.2;
E.nu            = 1;

E.acon          = pi/2-pi/6;
E.ucon          = 0.95;

O               = struct;
O.max_trial     = 1000;
O.max_steps     = 1000;
O.plot_every    = 6;
O.simCoarse     = 6;
O.periods       = 15;

env             = struct;
env.E           = E;
env.O           = O;

save('env.mat','env');