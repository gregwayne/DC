opts                = struct;
opts.rng            = [];
opts.max_trials     = 100;
opts.max_steps      = 5000;
opts.memory_based   = 1;
opts.single_arena   = 0;

[results, opts]     = RunTest(opts);
logging             = results.logging;

%% Compute cost histogram based on logging
costs               = zeros(opts.max_trials,3);
n_collisions        = zeros(opts.max_trials,1);
for trial=1:opts.max_trials
        
    env                 = results.envs{trial};
    
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
            n_collisions(trial) = n_collisions(trial) + 1;
        end
        
    end
    
    costs(trial,:)      = costs(trial,:)/logging{trial}{4};
    
end

max_costs    = max(costs,[],1);

figure();
subplot(3,1,1);
hist(costs(:,1),20);
subplot(3,1,2);
hist(costs(:,2),20);
subplot(3,1,3);
hist(costs(:,3),20);

figure();
hist(n_collisions);

