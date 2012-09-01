addpath minFunc/;
Forward         = @(x,u) SimDynamics(E,x,u,O.simCoarse);
options.Method  = 'lbfgs'; 
options.maxIter = 100;	
options.display = 'off'; 