function logging = GameMain(gameName)
                                  
    %% Globals
    global quit_sim new_sim record_sim;
    quit_sim    = 0;
    new_sim     = 0;
    record_sim  = 0;
    
    global env;
    global E;
    global O;
            
    mycontrols;     
    load_networks;

    %% Graphics and Recording
    global G;
    G           = [];    
    logging     = {};
    recording   = 0;
    
    %% Counters
    gameTerminatorCnt   = TimeCounter(env.O.max_steps);
    gamePlotterCnt      = TimeCounter(env.O.plot_every);
           
    for trial=1:env.O.max_trial
                
        if quit_sim
            break;
        end
                    
        if strcmp(gameName,'Obstacle')
            env.E.obstacles = [];
        end
        [env,x]             = init_sim(env);
        
        disp(sprintf('Trial %d', trial));
                
        t_step = 0;
        
        if strcmp(gameName,'Obstacle')
            while 1
                map                 = zeros(env.O.mapN,1);
                obs_delta           = bsxfun(@minus,env.E.obstacles,x(1:2));
                obs_dists           = sqrt(sum(obs_delta.^2,1));
                obs_goal_dists      = sqrt(sum(env.E.obstacles.^2,1));
                if ~((min(obs_dists) <= (env.E.disk + env.E.tl + 1)) ...
                    || (min(obs_goal_dists) <= 2*env.E.disk + 1))

                    % not inside disk
                    break;
                else
                    env.E.obstacles     = [];
                    [env,x]             = init_sim(env);
                end
            end
        end
        
        % logging saves environment data, 
        % state data, and control data
        logging{trial}  = {env,zeros(4,env.O.max_steps),...
                                        zeros(1,env.O.max_steps)};
        
        while 1 
            
            t_step = t_step + 1;
            
            if quit_sim
                if recording
                    close(writerObj);  
                end
                
                close all;                
                break;
            end
            
            if new_sim
                gameTerminatorCnt(1);  
                new_sim = 0;
                break;
            end
                        
            %% Simulation Code Here
            if gameTerminatorCnt(0) || TerminateCondition(env.E, x)
                gameTerminatorCnt(1);  
                break;
            end
                                    
                switch gameName 
                    
                    case 'Obstacle'
                        psense  = SenseProprioceptive(x);
                        gsense  = SenseGoal(x);        
                        osense  = SenseObstacles(env,x);
                        
                        m2      = FProp(hcontroller,[psense;gsense;osense]);                        
                        x_fic   = ConvertBearingToVirtualPosition(env.E,m2,x);
                        ix      = InternalStateObstacle(x_fic);
                        
                        map     = zeros(env.O.mapN,1);
                        [map,env.E.obstacles_detected] = ShootBeams(env,x,map,1);
                                                
                    case 'Truck'
                        ix      = InternalStateTruck(x);
                    otherwise
                        error('Unknown Game');
                        
                end
            
                m1      = lcontroller.FProp(ix);
                
                logging{trial}{2}(:,t_step) = x;
                logging{trial}{3}(:,t_step) = m1;               

                if strcmp(gameName,'Obstacle')
                    x       = SimDynamics(env,x,m1,max(1,ceil(get(uictrls.speedslider,'Value'))));
                    %x       = SimDynamics(env,x,m1,env.O.simCoarse);                                                
                else
                    x       = TruckDynamics(env,x,m1,max(1,ceil(get(uictrls.speedslider,'Value'))));
                end
                    
            jiggle_on = get(uictrls.jiggletoggle,'Value');
            if strcmp(gameName,'Obstacle') && jiggle_on

                [env,G] = JiggleObstacles(env,G);

            end
            
            %% Graphics Here
            if gamePlotterCnt(0)
            
                simdisp_on  = get(uictrls.simdisptoggle,'Value');

                if simdisp_on
                    
                    % Draw Simulation
                    if strcmp(gameName,'Obstacle')
                        G   = SimAnimation(env, G, x, m1);

                        DrawSubgoal(env,G,m2,x);
                        if strcmp(gameName,'Obstacle')                                                                        
                            if ~isempty(G.lineplan)
                                delete(G.lineplan);
                                G.lineplan = [];
                            end
                        end     
                    else
                        G   = TruckAnimation(env, G, x, m1);
                    end
                    
                end
                                                
                if record_sim
                    [fname, pname]  = uiputfile('*.avi');
                    writerObj       = VideoWriter(strcat(pname,fname),'Motion JPEG AVI');
                    writerObj.FrameRate = 60;
                    open(writerObj);
                    record_sim      = false;
                    recording       = 1;
                end
                drawnow;

                if recording
                    set(0,'CurrentFigure',G.fig);
                    writeVideo(writerObj,getframe);
                end
                
            end 
                                                    
        end
                   
    end
            
end