function [results,options] = RunTest(options,env)

    addpath('..');
    addpath('../Obstacle');
    addpath('../Optimal');
    
    if isempty(env)
        run ../Obstacle/MakeParameters;
        if isfield(options,'nobstacles')
            env.E.nobstacles = options.nobstacles;
        end
        env.E.obstacles = [];    
    end
    results     = struct;
    
    if isempty(options.rng)
        rng('shuffle');        
        options.rng     = rng;
    else        
        rng(options.rng);        
    end
             
    d_max               = 500;
    
    run ../load_networks;
    Forward             = @(x,m) HForward(env,x,m,lcontroller);
    addpath('../minFunc/');
    optPars.Method      = 'lbfgs'; 
    optPars.maxIter     = 1e5;
    optPars.maxFunEvals = 1e5;
    optPars.display     = 'on';
    warning off;
        
    %% Graphics and Recording  
    logging             = {};  
    if options.single_arena

        while 1
            [env,x]             = init_sim(env,d_max);
            obs_goal_dists      = sqrt(sum(env.E.obstacles.^2,1));
            if (min(obs_goal_dists) > 2*env.E.disk + 1)
                break;
            else
                env.E.obstacles = [];
            end
        end
    
        G                   = SimAnimation(env, [], x, 0);
        axis([-500 500 -500 500]);
        drawnow;
        close all;
    else        
        results.envs = {};
    end

    trial = 0;
    while trial < options.max_trials     
        trial           = trial + 1;
        disp(sprintf('Trial %d',trial));
        
        t_step          = 1;
        logging{trial}  = {env,zeros(4,1e4),...
                            zeros(1,1e4),0};
                        
        if ~options.single_arena
            while 1
                env.E.obstacles     = [];
                [env,x]             = init_sim(env,d_max);
                obs_delta           = bsxfun(@minus,env.E.obstacles,x(1:2));
                obs_dists           = sqrt(sum(obs_delta.^2,1));
                
                obs_goal_dists      = sqrt(sum(env.E.obstacles.^2,1));
                if (min(obs_goal_dists) > 2*env.E.disk + 1) ...
                    && min(obs_dists) >= (env.E.disk + env.E.tl + 5) ...
                    && ~TerminateCondition(env.E,x)
                
                    break;
                end
            end
            
            results.envs{trial} = env;
        else
        
            while 1
                [env,x]             = init_sim(env,d_max);
                obs_delta           = bsxfun(@minus,env.E.obstacles,x(1:2));
                obs_dists           = sqrt(sum(obs_delta.^2,1));
                if ~(min(obs_dists) <= (env.E.disk + env.E.tl + 5) ...
                        || TerminateCondition(env.E,x))
                    break;
                end
            end
            
        end
        
        t_step = 0;
                
        while 1 
            
            t_step      = t_step + 1;

            logging{trial}{2}(:,t_step) = x;
            
            %% Simulation Code Here
            if ((t_step > options.max_steps) ...
                    || TerminateCondition(env.E,x));
                
                logging{trial}{4}  = t_step;
                break;
            end                                    

            if options.memory_based == 1
                
                psense  = SenseProprioceptive(x);
                gsense  = SenseGoal(x);        
                osense  = SenseObstacles(env,x);

                m2      = FProp(hcontroller,[psense;gsense;osense]);                        
                x_fic   = ConvertBearingToVirtualPosition(env.E,m2,x);
                ix      = InternalState(x_fic);

                m1      = lcontroller.FProp(ix);
                                
                x       = SimDynamics(env,x,m1,1);                
         
            elseif options.memory_based == 2
                
                ix      = InternalState(x);
                m1      = lcontroller.FProp(ix);
                
                x       = SimDynamics(env,x,m1,1);                                  
                
            else
                
                flgs        = [0,0;1,0];
                m2s         = 0.05*randn(3,options.siml);                
                [m2s,cst]   = ComputeOpenLoop(Forward,options.siml,x,m2s,...
                                @(x,p) LossX(env,x,p),...
                                @(m,p) LossM(env,m,p),...
                                    optPars,flgs);
                m2s         = reshape(m2s,[3,options.siml]);
                                
                [xs,m1s]    = ComputeMicroTrajectory(env,x,m2s,lcontroller);
                
                for t=1:size(xs,2)
                    logging{trial}{2}(:,t_step+t-1) = xs(:,t);
                    logging{trial}{3}(:,t_step+t-1) = m1s(:,t);
                end
                                                
                x           = xs(:,end);
                t_step      = t_step + size(xs,2) - 1;
                
            end
            
            logging{trial}{3}(:,t_step) = m1;   
                        
        end
                 
    end
    
    results.logging     = logging;
    if options.single_arena
        results.env         = env;
    end
                  
end