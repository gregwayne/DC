opts                = struct;
opts.rng            = [];
opts.max_trials     = 100;
opts.max_steps      = 6000;
opts.memory_based   = 1;
opts.single_arena   = 1;
opts.siml           = 10;

[results, opts]     = RunTest(opts,env); % make env for repeat
logging             = results.logging;
env                 = results.env;

env.E.obstacles_detected    = [];
x                           = zeros(4,1);
    
G                   = SimAnimation(env, [], x, 0);
axis([-500 500 -500 500]);
delete(G.left_wheel);
delete(G.right_wheel);
delete(G.trailer);
delete(G.cab);
delete(G.link);
delete(G.plan);
delete(G.lineplan);
for i=1:env.O.mapN
    delete(G.beam(i));
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
            n_collisions(trial) = 1;
        end
        
    end
    
    costs(trial,:)      = costs(trial,:)/logging{trial}{4};
    
end

max_costs    = max(costs,[],1);

set(0,'CurrentFigure',G.fig);
set(gcf,'Position',[0 0 600 600]);
for trial=1:opts.max_trials
    xs     = logging{trial}{2}(1:2,1:logging{trial}{4});
    line(xs(1,:),xs(2,:),'LineWidth',2,...
        'Color',rand(3,1));
    drawnow;
end  

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
a=bar(1:opts.max_trials,costs(:,3));
set(a,'FaceColor',[1 0 0]);
hold on;
b=bar(1:opts.max_trials,costs(:,2));
set(b,'FaceColor',[0 0 1]);
c=get(b,'Children');
set(c,'FaceAlpha',0.5);
xlim([0 opts.max_trials+1]);
set(gca,'YTick',[]);

figure();
hist(n_collisions);

disp(sprintf('mean of run %f',mean(costs(:,3))));


