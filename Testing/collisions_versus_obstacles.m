opts                = struct;
opts.rng            = [];
opts.max_trials     = 100;
opts.max_steps      = 3000;
opts.memory_based   = 1;
opts.single_arena   = 0;

rounds              = 20;

collisions          = zeros(opts.max_trials,rounds);

for k=1:rounds

    opts.nobstacles     = 5*k;
    disp(sprintf('Obstacle Count is %d',opts.nobstacles));    

    [results, opts]     = RunTest(opts,[]); 
    logging             = results.logging;

    %% Compute collisions
    n_collisions        = zeros(opts.max_trials,1);
    for trial=1:opts.max_trials

        env     = results.envs{trial};

        for t=1:logging{trial}{4}

            x_t             = logging{trial}{2}(:,t);

            obstacle_dists  = sqrt(sum(bsxfun(@minus,env.E.obstacles,x_t(1:2)).^2,1));
            min_dist        = min(obstacle_dists);
            if (min_dist <= env.E.disk)
                n_collisions(trial) = 1;
            end

        end

        collisions(trial,k) = n_collisions(trial);
        
    end

end

