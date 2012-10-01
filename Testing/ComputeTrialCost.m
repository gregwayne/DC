function [costs,collision]  = ComputeTrialCost(env,logging,trial)

    costs               = zeros(3,1);
    collision           = 0;

    for t=1:logging{trial}{4}

        x_t             = logging{trial}{2}(:,t);
        gsense          = SenseGoal(x_t);
        goal_cost       = gsense(1);
        costs(1)        = costs(1) + goal_cost;
        obstacle_cost   = env.O.rel_cost*ObstacleCost(env,x_t);
        costs(2)        = costs(2) + obstacle_cost;
        costs(3)        = costs(3) + goal_cost + obstacle_cost;

        obstacle_dists  = sqrt(sum(bsxfun(@minus,env.E.obstacles,x_t(1:2)).^2,1));
        min_dist        = min(obstacle_dists);
        if (min_dist <= env.E.disk)
            collision   = 1;
        end

    end

end