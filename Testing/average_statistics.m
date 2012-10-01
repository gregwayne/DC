opts                = struct;
opts.rng            = [];
opts.max_trials     = 1000;
opts.max_steps      = 6000;
opts.memory_based   = 2;
opts.single_arena   = 0;

[results, opts]     = RunTest(opts,[]); 
logging             = results.logging;

%% Compute cost histogram based on logging
costs               = zeros(opts.max_trials,3);
n_collisions        = zeros(opts.max_trials,1);
for trial=1:opts.max_trials
    
    env     = results.envs{trial};
        
    for t=1:logging{trial}{4}
        
        x_t             = logging{trial}{2}(:,t);
        gsense          = SenseGoal(x_t);
        goal_cost       = gsense(1);
        costs(trial,1)  = costs(trial,1) + goal_cost;
        obstacle_cost   = env.O.rel_cost*ObstacleCost(env,x_t);
        costs(trial,2)  = costs(trial,2) + obstacle_cost;
        costs(trial,3)  = costs(trial,3) + goal_cost + obstacle_cost;
    
        obstacle_dists  = sqrt(sum(bsxfun(@minus,env.E.obstacles,x_t(1:2)).^2,1));
        min_dist        = min(obstacle_dists);
        if (min_dist <= env.E.disk)
            n_collisions(trial) = 1;
        end
        
    end
    
    costs(trial,:)      = costs(trial,:)/logging{trial}{4};
    
end

max_costs    = max(costs,[],1);

[N3,X]  = hist(costs(:,3),30);
maxX    = max(X);
XP      = linspace(0,maxX,30);

figure();
subplot(3,1,1);
N1=hist(costs(:,1),XP);
bar(XP,N1/sum(N1));
text(mean(costs(:,1)), max(N1/sum(N1)),'.','FontSize',50,'Color','r');
subplot(3,1,2);
N2=hist(costs(:,2),XP);
bar(XP,N2/sum(N2));
text(mean(costs(:,2)), max(N2/sum(N2)),'.','FontSize',50,'Color','r');
subplot(3,1,3);
N3=hist(costs(:,3),XP);
bar(XP,N3/sum(N3));
text(mean(costs(:,3)), max(N3/sum(N3)),'.','FontSize',50,'Color','r');

figure();
hist(n_collisions);

disp(sprintf('mean of run %f',mean(costs(:,3))));

figure();
hist(n_collisions);

first_dists     = zeros(opts.max_trials,1);
successes       = zeros(opts.max_trials,1);

for trial=1:opts.max_trials
    
    first_x             = logging{trial}{2}(:,1);
    first_dists(trial)  = sqrt(sum(first_x(1:2).^2));
    last_x              = logging{trial}{2}(:,logging{trial}{4});
    if (sqrt(sum(last_x(1:2).^2)) <= results.envs{trial}.E.disk+5)
        successes(trial)    = 1;
    end
    
end

scatter(first_dists,successes);

save(strcat('results+',num2str(opts.memory_based),'.mat'),'results');