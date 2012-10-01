load 'env.mat';
addpath('..');
[env,x]     = init_sim(env);
E           = env.E;

Forward     = @(x,u) SimDynamics(E,x,u,1);

u           = rand-0.5;
[Jx,Ju]     = NumericalJacobian(Forward,x,u);