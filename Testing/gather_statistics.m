opts                = struct;
opts.rng            = [];
opts.max_trials     = 100;
opts.max_steps      = 5000;
opts.memory_based   = 1;
opts.single_arena   = 1;

[results, opts]     = RunTest(opts);
logging             = results.logging;
env                 = results.env;

env.E.obstacles_detected    = [];
x                           = randn(4,1);

G                           = {};

for scr=1:3
    
    G{scr}      = SimAnimation(env, [], x, 0);
    axis([-500 500 -500 500]);
    delete(G{scr}.left_wheel);
    delete(G{scr}.right_wheel);
    delete(G{scr}.trailer);
    delete(G{scr}.cab);
    delete(G{scr}.link);
    delete(G{scr}.plan);
    delete(G{scr}.lineplan);
    for i=1:env.O.mapN
        delete(G{scr}.beam(i));
    end
    
    text(-410,-410,num2str(scr),'FontSize',100);

end

%% Compute cost histogram based on logging
costs               = zeros(opts.max_trials,3);
n_collisions        = zeros(opts.max_trials,1);
for trial=1:opts.max_trials
        
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

for scr=1:3

    set(0,'CurrentFigure',G{scr}.fig);
    for trial=1:opts.max_trials
        xs     = logging{trial}{2}(1:2,1:logging{trial}{4});
        line(xs(1,:),xs(2,:),'LineWidth',2,...
            'Color',(1-(costs(trial,scr)/max_costs(scr)))*ones(3,1));
        drawnow;
    end  

end

[N3,X]  = hist(costs(:,3),20);
maxX    = max(X);
XP      = linspace(0,maxX,20);

figure();
subplot(3,1,1);
hist(costs(:,1),XP);
subplot(3,1,2);
hist(costs(:,2),XP);
subplot(3,1,3);
hist(costs(:,3),XP);

figure();
hist(n_collisions);

